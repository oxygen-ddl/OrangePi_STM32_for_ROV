#pragma once
#include <array>
#include <cstdint>
#include <vector>
extern "C" {
#include "protocol_pack.h"
}

struct ProtocolV1Packer {
    static std::vector<uint8_t> packPWM(const std::array<uint16_t,8>& pwm){
        std::vector<uint8_t> buf(32); // 14 + 16 + 2 = 32
        uint16_t n = 0;
        protocol_pack_pwm(pwm.data(), buf.data(), (uint16_t)buf.size(), &n);
        buf.resize(n);
        return buf;
    }
    static std::vector<uint8_t> packHeartbeat(){
        std::vector<uint8_t> buf(16); // 14 + 0 + 2 = 16
        uint16_t n = 0;
        protocol_pack_heartbeat(buf.data(), (uint16_t)buf.size(), &n);
        buf.resize(n);
        return buf;
    }
};
