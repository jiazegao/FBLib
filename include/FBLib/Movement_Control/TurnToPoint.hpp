#pragma once

#include "FBLib/Movement_Control/Motion.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

enum class TurnDirection {
    CCW,         // counter-clockwise always
    CW,          // clockwise always
    SHORTEST     // shortest angular path (default)
};

enum class SwingSide {
    Left,   // left motors stationary, right motors turn
    Right   // right motors stationary, left motors turn
};

// ============================================================================
// TurnToPoint utility functions
// ============================================================================

/// Resolve signed angular error given a desired turn direction
float resolveTurnError(float rawErrorRad, TurnDirection direction);

/// Heading error to face a target point from current pose
float headingToPoint(const Pose& current, float targetX, float targetY);

/// Heading error with an angular offset
/// @param offsetDeg  counter-clockwise offset in degrees (standard math)
float headingToPointOffset(const Pose& current, float targetX, float targetY,
                            float offsetDeg);

/// Check if heading error is within tolerance (both in radians, internal use)
bool headingWithinTolerance(float headingErrorRad, float toleranceRad);

// ============================================================================
// Parameter structs
// ============================================================================

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};          // > 0: never slow below this; exits on passing the target (chaining)
    float targetTolerance{2.0f};   // VEX degrees
};

struct TurnToPointParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{2.0f};   // VEX degrees
    float offset{0.0f};            // clockwise offset in VEX degrees (180 = face away from the point)
};

struct SwingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{2.0f};   // VEX degrees
};

// ============================================================================
// TurnDirectionResolver
// ============================================================================
//
// Applies a forced CW/CCW direction only until the robot first crosses the
// target. Forcing it on every tick turns any overshoot into another full
// revolution; after the first crossing the shortest path is used.
// ============================================================================

class TurnDirectionResolver {
public:
    explicit TurnDirectionResolver(TurnDirection direction) : mDirection(direction) {}

    /// @param rawErrorRad  shortest signed error to the target, [-pi, pi]
    float resolve(float rawErrorRad);

    /// True if the last resolve() call saw the robot pass the target.
    bool crossedTarget() const { return mCrossed; }

private:
    TurnDirection mDirection;
    bool mSettling{false};
    bool mHasPrevious{false};
    bool mCrossed{false};
    float mPreviousRaw{0.0f};
};

// ============================================================================
// Turn motions
// ============================================================================

/// Turn in place to an absolute heading (standard-math radians).
class TurnToHeadingMotion : public Motion {
public:
    TurnToHeadingMotion(float targetHeadingRad, const TurntoHeadingParams& params);
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTarget;
    TurntoHeadingParams mParams;
    TurnDirectionResolver mResolver;
};

/// Turn in place to face a field coordinate (optionally with an offset).
class TurnToPointMotion : public Motion {
public:
    TurnToPointMotion(float x, float y, const TurnToPointParams& params);
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTargetX;
    float mTargetY;
    TurnToPointParams mParams;
    TurnDirectionResolver mResolver;
};

/// Pivot around one stationary side of the drive to an absolute heading.
class SwingMotion : public Motion {
public:
    SwingMotion(float targetHeadingRad, SwingSide side, const SwingParams& params);
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTarget;
    SwingSide mSide;
    SwingParams mParams;
    TurnDirectionResolver mResolver;
};

}  // namespace FBLIB
