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

/// Time-sampled trapezoidal profile (triangular when maxVel is unreachable).
/// Invalid input (non-positive distance, speed, accel or dt) yields a single
/// point at rest.
std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt = 0.01f);

/// Time-sampled jerk-limited (7-phase S-curve) profile. For moves too short to
/// reach maxVel the peak speed is lowered so the profile still ends at rest
/// exactly at `distance`. maxJerk <= 0 falls back to the trapezoidal profile.
std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                          float maxAccel, float maxJerk,
                                          float dt = 0.01f);

/// Speed allowed `position` inches into a trapezoidal move of length
/// `distance` that starts and ends at `edgeVel` (use a small non-zero value
/// for a feedback controller, or it is commanded to stand still at the start).
float trapezoidalVelocityAt(float position, float distance, float maxVel,
                            float maxAccel, float edgeVel = 0.0f);

}  // namespace FBLIB
