#pragma once
#ifndef ROVCTRL_UTILS_DETAIL_DOF_HPP
#define ROVCTRL_UTILS_DETAIL_DOF_HPP

#include <array>
#include <string>

namespace rovctrl::utils::detail {

// DOF 名称 -> index，固定顺序 [Fx,Fy,Fz,Mx,My,Mz]
inline int dof_index(const std::string& name)
{
    if      (name == "Fx") return 0;
    else if (name == "Fy") return 1;
    else if (name == "Fz") return 2;
    else if (name == "Mx") return 3;
    else if (name == "My") return 4;
    else if (name == "Mz") return 5;
    return -1;
}

inline bool any_active(const std::array<bool, 6>& a)
{
    for (bool v : a) {
        if (v) return true;
    }
    return false;
}

} // namespace rovctrl::utils::detail

#endif
