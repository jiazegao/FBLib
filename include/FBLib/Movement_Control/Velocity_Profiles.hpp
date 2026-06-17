#pragma once

#include <vector>

namespace FBLIB {

struct MotionProfile {
    float maxVelocity;
    float maxAccel;
    float maxJerk;
    float distance;
    float duration;
};

struct ProfilePoint {
    float time{0.0f};
    float velocity{0.0f};
    float position{0.0f};
};

std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt = 0.01f);

std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                          float maxAccel, float maxJerk,
                                          float dt = 0.01f);

}  // namespace FBLIB
