#pragma once
#include "pros/rtos.hpp"
#include "flash/util/types.hpp"

namespace FBLib {

inline Number nowSeconds() {
    return static_cast<Number>(pros::millis()) / 1000.0;
}

} // namespace FBLib