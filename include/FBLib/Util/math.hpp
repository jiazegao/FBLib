#pragma once
#include <cmath>
#include <algorithm>
#include "FBLib/types.hpp"

namespace FBLib {
  // clamp value into [lo, hi]
inline Number clamp(Number v, Number lo, Number hi) {
    return std::max(lo, std::min(v, hi));
}

// returns -1, 0, or 1
inline int sign(Number v) {
    return (v > 0) - (v < 0);
}

// floating compare
inline bool nearlyEqual(Number a, Number b, Number eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

// keep value within [-limit, limit]
inline Number clampAbs(Number v, Number limit) {
    return clamp(v, -limit, limit);
}

} // namespace FBLib