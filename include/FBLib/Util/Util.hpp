#pragma once
#include <stdint.h>

namespace FBLib {
    using Number = double;

    struct Pose {
        Number x{0};
        Number y{0};
        Number theta{0}; // in radians
    };
}