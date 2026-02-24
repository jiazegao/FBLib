#pragma once
#include <cmath>
#include "flash/util/types.hpp"

namespace FBLib {

constexpr Number PI = 3.14159265358979323846;

// wrap angle to (-pi, pi]
inline Number wrapRad(Number a) {
    a = std::fmod(a + PI, 2 * PI);
    if (a < 0) a += 2 * PI;
    return a - PI;
}

// shortest signed difference (target - current), result in (-pi, pi]
inline Number angleDiffRad(Number target, Number current) {
    return wrapRad(target - current);
}

} // namespace FBLib