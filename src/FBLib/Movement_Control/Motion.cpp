#include "FBLib/Movement_Control/Motion.hpp"

#include <algorithm>
#include <cmath>

namespace FBLIB {

void Motion::start(const Pose& pose, const MotionContext& ctx) {
    (void)pose;
    (void)ctx;
}

MotionOutput mixDrive(float lateral, float angular, float maxSpeed) {
    // Differential drive: omega = (vRight - vLeft) / trackWidth, so turning
    // counter-clockwise (positive angular) needs the right side faster.
    float left = lateral - angular;
    float right = lateral + angular;

    float limit = std::fabs(maxSpeed);
    if (limit > 127.0f) limit = 127.0f;
    float largest = std::max(std::fabs(left), std::fabs(right));
    if (largest > limit && largest > 0.0f) {
        float scale = limit / largest;
        left *= scale;
        right *= scale;
    }

    MotionOutput out;
    out.left = left;
    out.right = right;
    return out;
}

float applyMinSpeed(float command, float minSpeed) {
    float floor = std::fabs(minSpeed);
    if (command > 0.0f && command < floor) return floor;
    if (command < 0.0f && command > -floor) return -floor;
    return command;
}

}  // namespace FBLIB
