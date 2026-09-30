#pragma once

#include "FBLib/Util/Util.hpp"
#include "FBLib/Util/pid.hpp"

namespace FBLIB {

// ============================================================================
// Motion — common interface for every autonomous movement
// ============================================================================
//
// Each movement type (move to point, turn, arc, boomerang, RAMSETE, ...) is a
// Motion subclass that turns the current pose into left/right drive commands.
// Motions contain only the control math: they never touch hardware, tasks,
// timeouts or cancellation. Chassis owns those and calls update() every
// 10 ms, both on the robot and during dry-run simulation.
//
// Conventions:
//   - Pose.theta is standard math: radians, 0 = +X, counter-clockwise positive.
//   - Drive commands are -127..127, positive drives that side forward.
//   - A positive angular command turns the robot counter-clockwise, i.e. it
//     drives the RIGHT side faster (see mixDrive()).
// ============================================================================

/// What a motion needs besides the pose. Supplied by Chassis every tick.
struct MotionContext {
    PID& lateralPID;          // distance controller (error in inches)
    PID& angularPID;          // heading controller (error in radians)
    float trackWidth;         // inches, left-to-right wheel spacing
    float maxSpeedInPerSec;   // drivetrain top linear speed, inches/second
};

/// Result of one control tick.
struct MotionOutput {
    float left{0.0f};       // left side command, -127..127
    float right{0.0f};      // right side command, -127..127
    bool done{false};       // settled, exited early, or crossed the target
    float remaining{-1.0f}; // wheel travel still needed (inches); Chassis ends a
                            // motion that stops making progress on this number.
                            // Negative = not reported (no stall exit).
};

class Motion {
public:
    virtual ~Motion() = default;

    /// Called once, immediately before the first update(), with the pose at
    /// that moment. Motions that are defined relative to where the robot is
    /// (e.g. moveDistance) capture their target here.
    virtual void start(const Pose& pose, const MotionContext& ctx);

    /// Compute one control tick. `dt` is the time since the previous tick in
    /// seconds.
    virtual MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) = 0;
};

// ============================================================================
// Shared helpers for motion implementations
// ============================================================================

/// Distance inside which point-chasing motions stop steering at the target.
/// Very close to a point, the bearing to it swings wildly with tiny position
/// errors; steering at it there makes the robot spin or orbit the target.
constexpr float MOTION_CLOSE_RANGE = 7.5f;  // inches

/// Mix lateral and angular commands into left/right. Positive angular turns
/// counter-clockwise. If either side would exceed maxSpeed, both are scaled
/// down together so the commanded curvature is preserved.
MotionOutput mixDrive(float lateral, float angular, float maxSpeed);

/// Raise a non-zero command to at least minSpeed in magnitude (sign kept).
float applyMinSpeed(float command, float minSpeed);

}  // namespace FBLIB
