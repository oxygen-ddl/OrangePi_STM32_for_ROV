#include "gateway/IPC/intent/intent_keyboard_source.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <cmath> // std::abs

namespace comm_gcs::ipc::intent {

namespace {

inline double clampd(double v, double lo, double hi) noexcept
{
    return std::max(lo, std::min(v, hi));
}

inline std::uint64_t ms_to_ns(std::uint64_t ms) noexcept
{
    return ms * 1000ULL * 1000ULL;
}

// Dof index order: surge, sway, heave, roll, pitch, yaw
enum DofIdx : std::size_t {
    kSurge = 0,
    kSway  = 1,
    kHeave = 2,
    kRoll  = 3,
    kPitch = 4,
    kYaw   = 5,
    kNumDof = 6
};

inline void zero_dirs(double dir[kNumDof]) noexcept
{
    for (std::size_t i = 0; i < kNumDof; ++i) dir[i] = 0.0;
}

inline void zero_except(double dir[kNumDof], std::size_t keep) noexcept
{
    for (std::size_t i = 0; i < kNumDof; ++i) {
        if (i != keep) dir[i] = 0.0;
    }
}

static inline bool dof_nonzero_eps(const shared::msg::DofCommand& d) noexcept
{
    // 避免 double 残差导致一直被认为“非零”
    constexpr double eps = 1e-6;
    return (std::abs(d.surge) > eps) || (std::abs(d.sway)  > eps) || (std::abs(d.heave) > eps) ||
           (std::abs(d.roll)  > eps) || (std::abs(d.pitch) > eps) || (std::abs(d.yaw)   > eps);
}

} // namespace

bool IntentKeyboardSource::init(const Config& cfg)
{
    shutdown();

    cfg_     = cfg;
    enabled_ = cfg_.enable;

    if (!enabled_) {
        initialized_ = false;
        return true;
    }

    if (!key_sub_.init(cfg_.key_sub)) {
        initialized_ = false;
        return false;
    }

    if (!intent_pub_.init(cfg_.intent_pub)) {
        initialized_ = false;
        return false;
    }

    intent_.clear_all();
    cmd_seq_ = 0;

    throttle_cmd_ = clampd(throttle_cmd_,
                           static_cast<double>(cfg_.throttle_min),
                           static_cast<double>(cfg_.throttle_max));

    for (std::size_t i = 0; i < kNumDof; ++i) {
        last_refresh_ns_[i] = 0;
        dir_[i] = 0.0;
    }

    stats_ = Stats{};
    initialized_ = true;
    return true;
}

void IntentKeyboardSource::shutdown() noexcept
{
    key_sub_.shutdown();
    intent_pub_.shutdown();

    enabled_     = false;
    initialized_ = false;

    cfg_ = Config{};
    intent_.clear_all();
    cmd_seq_ = 0;

    throttle_cmd_ = 0.0;
    for (std::size_t i = 0; i < kNumDof; ++i) {
        last_refresh_ns_[i] = 0;
        dir_[i] = 0.0;
    }

    stats_ = Stats{};
}

bool IntentKeyboardSource::tick(std::uint64_t now_mono_ns) noexcept
{
    if (!enabled_) return true;
    if (!initialized_) return false;

    clear_pulses_();

    // bounded drain to keep tick time predictable
    std::array<shared::msg::KeyEvent, 128> buf{};
    constexpr std::size_t kMaxDrainLoops = 8;

    std::size_t total = 0;
    for (std::size_t loop = 0; loop < kMaxDrainLoops; ++loop) {
        const std::size_t n = key_sub_.drain(buf.data(), buf.size());
        if (n == 0) break;

        for (std::size_t i = 0; i < n; ++i) {
            handle_event_(buf[i], now_mono_ns);
        }

        total += n;
        if (n < buf.size()) break;
    }

    stats_.rx_key_events += total;
    stats_.key_sub_stats = key_sub_.stats();

    compose_intent_(now_mono_ns);
    update_flags_();

    // ---- fill meta required by arbiter ttl policy ----
    intent_.version      = shared::msg::kControlIntentWireVersion;
    intent_.ttl_ms       = cfg_.ttl_ms;       // 0 => receiver default
    intent_.source_id    = cfg_.source_id;
    intent_.source_prio  = cfg_.source_prio;

    intent_.stamp_ns     = now_mono_ns;       // MUST be non-zero if stamp0_is_expired=true
    intent_.cmd_seq      = ++cmd_seq_;

    if (!intent_pub_.publish(intent_)) {
        return false;
    }

    stats_.tx_intents += 1;
    return true;
}

void IntentKeyboardSource::clear_pulses_() noexcept
{
    // Pulse-style fields reset each tick; handle_event_ may set them for this frame.
    intent_.request_exit = 0;

    intent_.estop        = 0;
    intent_.clear_estop  = 0;

    intent_.arm          = 0;
    intent_.disarm       = 0;

    // mode_request：按“按键触发一次请求”的脉冲语义处理（你们当前设计就是这个）
    intent_.mode_request = shared::msg::ControlMode::kNone;

    // flags 每 tick 由 update_flags_() 重算
    intent_.flags = 0;
}

inline void keep_only(std::uint64_t last[kNumDof], double dir[kNumDof], std::size_t keep) noexcept {
    for (std::size_t i = 0; i < kNumDof; ++i) {
        if (i != keep) { dir[i] = 0.0; last[i] = 0; }
    }
}


void IntentKeyboardSource::handle_event_(const shared::msg::KeyEvent& ev,
                                        std::uint64_t now_mono_ns) noexcept
{
    using comm_gcs::ipc::keys::TeleopAction;

    // 若你 mapper 实现不是这个函数名，把这一行替换为你实际接口即可：
    const TeleopAction a =
        comm_gcs::ipc::keys::map_key_event_to_action(ev, cfg_.mapper_cfg);

    switch (a) {
        case TeleopAction::None:
            return;

        // -------- discrete commands (pulse fields) --------
        case TeleopAction::Exit:
            intent_.request_exit = 1;
            intent_.estop        = 1;   // 关键：退出前先触发停机脉冲
            return;

        case TeleopAction::Help:
            // Help is local diagnostic; it does not enter ControlIntent.
            std::cerr
                << "\n===== Gateway Keyboard Intent Source =====\n"
                << "Hold-to-run:\n"
                << "  W/S surge +/-, A/D sway -/+, H/G heave +/-, Q/E yaw +/-,\n"
                << "  R/T roll +/-, F/V pitch +/-, M center, ESC exit\n"
                << "Throttle:\n"
                << "  =/+ up, -/_ down, throttle_cmd=" << throttle_cmd_ << "\n"
                << "Safety:\n"
                << "  Ctrl+Space E-STOP, Ctrl+M Clear E-STOP, ,/， arm, ./。 disarm\n"
                << "==========================================\n\n";
            return;

        case TeleopAction::Center:
            zero_dirs(dir_);
            for (std::size_t i = 0; i < kNumDof; ++i) last_refresh_ns_[i] = 0;
            return;

        case TeleopAction::EStop:
            intent_.estop = 1;
            return;

        case TeleopAction::ClearEStop:
            intent_.clear_estop = 1;
            return;

        case TeleopAction::Arm:
            intent_.arm = 1;
            return;

        case TeleopAction::Disarm:
            intent_.disarm = 1;
            return;

        case TeleopAction::ThrottleUp:
            throttle_cmd_ = clampd(throttle_cmd_ + static_cast<double>(cfg_.throttle_step),
                                   static_cast<double>(cfg_.throttle_min),
                                   static_cast<double>(cfg_.throttle_max));
            return;

        case TeleopAction::ThrottleDown:
            throttle_cmd_ = clampd(throttle_cmd_ - static_cast<double>(cfg_.throttle_step),
                                   static_cast<double>(cfg_.throttle_min),
                                   static_cast<double>(cfg_.throttle_max));
            return;

        // -------- single-motor test (discrete, pulse-to-executor) --------
        // Semantics:
        // - Key press triggers a motor test request.
        // - Execution (2s hold + auto-center) is done by upper layer (arbiter/executor),
        //   NOT by blocking the keyboard handler thread.
        case TeleopAction::MotorTest1:
        case TeleopAction::MotorTest2:
        case TeleopAction::MotorTest3:
        case TeleopAction::MotorTest4:
        case TeleopAction::MotorTest5:
        case TeleopAction::MotorTest6:
        case TeleopAction::MotorTest7:
        case TeleopAction::MotorTest8: {
            // Recommended: during motor test request, clear DOF refresh to keep the frame unambiguous.
            zero_dirs(dir_);
            for (std::size_t i = 0; i < kNumDof; ++i) last_refresh_ns_[i] = 0;

            // Map action -> motor_id
            std::uint8_t motor_id = 0;
            switch (a) {
                case TeleopAction::MotorTest1: motor_id = 1; break;
                case TeleopAction::MotorTest2: motor_id = 2; break;
                case TeleopAction::MotorTest3: motor_id = 3; break;
                case TeleopAction::MotorTest4: motor_id = 4; break;
                case TeleopAction::MotorTest5: motor_id = 5; break;
                case TeleopAction::MotorTest6: motor_id = 6; break;
                case TeleopAction::MotorTest7: motor_id = 7; break;
                case TeleopAction::MotorTest8: motor_id = 8; break;
                default: break;
            }
            if (motor_id == 0) return;

            // Build MotorTest command:
            intent_.motor_test.enable = 1;
            intent_.motor_test.motor_id = motor_id;
            intent_.motor_test.mode = 0;                 // 0=neutral+delta (recommended)
            intent_.motor_test.value = 0.6f;             // default normalized thrust, tune later
            intent_.motor_test.duration_ms = 2000;       // your requirement: 2s then auto stop/center
            intent_.motor_test.cmd_id = ++motor_test_cmd_seq_; // add a member seq counter

            intent_.flags |= shared::msg::kHasMotorTest;
            return;
        }

        case TeleopAction::MotorTestStop: {
            // Stop request: upper layer should immediately center the motor(s) / exit test mode.
            intent_.motor_test.enable = 0;
            intent_.motor_test.motor_id = 0;
            intent_.motor_test.mode = 0;
            intent_.motor_test.value = 0.0f;
            intent_.motor_test.duration_ms = 0;
            intent_.motor_test.cmd_id = ++motor_test_cmd_seq_;

            intent_.flags |= shared::msg::kHasMotorTest;
            return;
        }
 
        // -------- continuous DOF (hold refresh) --------
        case TeleopAction::SurgePos:
            dir_[kSurge] = +1.0;
            last_refresh_ns_[kSurge] = now_mono_ns;
            return;
        case TeleopAction::SurgeNeg:
            dir_[kSurge] = -1.0;
            last_refresh_ns_[kSurge] = now_mono_ns;
            return;

        case TeleopAction::SwayPos:
            dir_[kSway] = +1.0;
            last_refresh_ns_[kSway] = now_mono_ns;
            return;
        case TeleopAction::SwayNeg:
            dir_[kSway] = -1.0;
            last_refresh_ns_[kSway] = now_mono_ns;
            return;

        case TeleopAction::HeavePos:
            dir_[kHeave] = +1.0;
            last_refresh_ns_[kHeave] = now_mono_ns;
            return;
        case TeleopAction::HeaveNeg:
            dir_[kHeave] = -1.0;
            last_refresh_ns_[kHeave] = now_mono_ns;
            return;

        case TeleopAction::YawPos:
            dir_[kYaw] = +1.0;
            last_refresh_ns_[kYaw] = now_mono_ns;
            return;
        case TeleopAction::YawNeg:
            dir_[kYaw] = -1.0;
            last_refresh_ns_[kYaw] = now_mono_ns;
            return;

        case TeleopAction::RollPos:
            dir_[kRoll] = +1.0;
            last_refresh_ns_[kRoll] = now_mono_ns;
            zero_except(dir_, kRoll);   // legacy: roll clears other DOF
            return;
            
        case TeleopAction::RollNeg:
            dir_[kRoll] = -1.0;
            last_refresh_ns_[kRoll] = now_mono_ns;
            zero_except(dir_, kRoll);
            return;

        case TeleopAction::PitchPos:
            dir_[kPitch] = +1.0;
            last_refresh_ns_[kPitch] = now_mono_ns;
            zero_except(dir_, kPitch); // legacy: pitch clears other DOF
            return;
        case TeleopAction::PitchNeg:
            dir_[kPitch] = -1.0;
            last_refresh_ns_[kPitch] = now_mono_ns;
            zero_except(dir_, kPitch);
            return;
    }
}

void IntentKeyboardSource::compose_intent_(std::uint64_t now_mono_ns) noexcept
{
    const std::uint64_t timeout_ns = ms_to_ns(cfg_.hold_timeout_ms);

    // hold timeout: if no refresh recently, that DOF -> 0
    for (std::size_t i = 0; i < kNumDof; ++i) {
        const std::uint64_t t = last_refresh_ns_[i];
        if (t == 0) {
            dir_[i] = 0.0;
            continue;
        }
        if (now_mono_ns > t && (now_mono_ns - t) > timeout_ns) {
            dir_[i] = 0.0;
            last_refresh_ns_[i] = 0;
        }
    }

    const double s = clampd(throttle_cmd_,
                            static_cast<double>(cfg_.throttle_min),
                            static_cast<double>(cfg_.throttle_max));

    shared::msg::DofCommand cmd{};
    cmd.surge = dir_[kSurge] * s;
    cmd.sway  = dir_[kSway]  * s;
    cmd.heave = dir_[kHeave] * s;

    cmd.roll  = dir_[kRoll]  * s;
    cmd.pitch = dir_[kPitch] * s;
    cmd.yaw   = dir_[kYaw]   * s;

    intent_.teleop_dof_cmd = cmd;
}

void IntentKeyboardSource::update_flags_() noexcept
{
    std::uint32_t f = 0;

    // ----- 退出请求（pulse）-----
    if (intent_.request_exit) {
        f |= kHasExitRequest;
    }

    // ----- 急停 / 解急停（pulse）-----
    if (intent_.estop || intent_.clear_estop) {
        f |= kHasEStopCmd;
    }

    // ----- ARM / DISARM（pulse）-----
    if (intent_.arm || intent_.disarm) {
        f |= kHasArmCmd;
    }

    // ----- 模式请求（pulse）-----
    if (intent_.mode_request != shared::msg::ControlMode::kNone) {
        f |= kHasModeRequest;
    }

    // ----- Teleop DOF（state）-----
    if (dof_nonzero_eps(intent_.teleop_dof_cmd)) {
        f |= kHasTeleopDof;
    }

    intent_.flags = f;
}


} // namespace comm_gcs::ipc::intent
