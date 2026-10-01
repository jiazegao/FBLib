#include "FBLib/Movement_Control/TurnToPoint.hpp"

#include <cmath>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// TurnToPoint — rotational movement utilities
// ============================================================================

/// Resolve the signed angular error given a desired turn direction.
/// Positive = CCW, Negative = CW.
float resolveTurnError(float rawErrorRad, TurnDirection direction) {
    switch (direction) {
    case TurnDirection::CCW:
        if (rawErrorRad < 0.0f) rawErrorRad += TWO_PI;
        break;
    case TurnDirection::CW:
        if (rawErrorRad > 0.0f) rawErrorRad -= TWO_PI;
        break;
    case TurnDirection::SHORTEST:
    default:
        break;
    }
    return rawErrorRad;
}

/// Compute heading error to face a target point from current pose.
float headingToPoint(const Pose& current, float targetX, float targetY) {
    float desiredHeadingRad = bearingToPoint(current, targetX, targetY);
    return angleDiffRad(current.theta, desiredHeadingRad);
}

/// Compute heading error with an angular offset (positive = CCW).
float headingToPointOffset(const Pose& current, float targetX, float targetY,
                            float offsetDeg) {
    float desiredHeadingRad = bearingToPoint(current, targetX, targetY) + degToRad(offsetDeg);
    return angleDiffRad(current.theta, desiredHeadingRad);
}

/// Check if heading error is within tolerance.
bool headingWithinTolerance(float headingErrorRad, float toleranceRad) {
    return std::fabs(headingErrorRad) < toleranceRad;
}

// ============================================================================
// TurnDirectionResolver
// ============================================================================

float TurnDirectionResolver::resolve(float rawErrorRad) {
    mCrossed = false;
    if (mHasPrevious && ((rawErrorRad > 0.0f) != (mPreviousRaw > 0.0f))) {
        // The shortest-path error changes sign either when the robot passes
        // the target (small error) or the point opposite it (|error| ~ pi).
        // Either way the forced direction has done its job: from here the
        // shortest path IS the requested direction, or the robot overshot.
        mSettling = true;
        mCrossed = std::fabs(rawErrorRad) < HALF_PI;
    }
    mPreviousRaw = rawErrorRad;
    mHasPrevious = true;

    if (mDirection == TurnDirection::SHORTEST || mSettling) return rawErrorRad;
    return resolveTurnError(rawErrorRad, mDirection);
}

// ============================================================================
// Shared turn controller
// ============================================================================

namespace {

struct TurnCommand {
    float angular;  // CCW-positive command, limited to maxSpeed
    float error;    // resolved heading error (rad)
    bool done;
};

TurnCommand computeTurn(float rawErrorRad, TurnDirectionResolver& resolver, float maxSpeed,
                        float minSpeed, float toleranceDeg, float dt, const MotionContext& ctx) {
    TurnCommand cmd{};
    cmd.error = resolver.resolve(rawErrorRad);
    cmd.angular = clamp(ctx.angularPID.update(cmd.error, dt), -maxSpeed, maxSpeed);
    cmd.angular = applyMinSpeed(cmd.angular, minSpeed);

    bool settled = std::fabs(cmd.error) < degToRad(toleranceDeg);
    // With a minimum speed the robot can't stop on the heading; end once it
    // passes it so a chained motion takes over.
    bool passed = minSpeed > 0.0f && resolver.crossedTarget();
    cmd.done = settled || passed;
    return cmd;
}

MotionOutput inPlaceTurn(const TurnCommand& cmd, float maxSpeed, float trackWidth) {
    MotionOutput out = mixDrive(0.0f, cmd.angular, maxSpeed);
    out.done = cmd.done;
    out.remaining = std::fabs(cmd.error) * trackWidth * 0.5f;  // wheel arc length
    return out;
}

}  // namespace

// ============================================================================
// TurnToHeadingMotion
// ============================================================================

TurnToHeadingMotion::TurnToHeadingMotion(float targetHeadingRad, const TurntoHeadingParams& params)
    : mTarget(targetHeadingRad), mParams(params), mResolver(params.direction) {}

MotionOutput TurnToHeadingMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    TurnCommand cmd = computeTurn(angleDiffRad(pose.theta, mTarget), mResolver, mParams.maxSpeed,
                                  mParams.minSpeed, mParams.targetTolerance, dt, ctx);
    return inPlaceTurn(cmd, mParams.maxSpeed, ctx.trackWidth);
}

// ============================================================================
// TurnToPointMotion
// ============================================================================

TurnToPointMotion::TurnToPointMotion(float x, float y, const TurnToPointParams& params)
    : mTargetX(x), mTargetY(y), mParams(params), mResolver(params.direction) {}

MotionOutput TurnToPointMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    // Offset is clockwise in VEX degrees; standard-math angles grow CCW.
    float desired = bearingToPoint(pose, mTargetX, mTargetY) - degToRad(mParams.offset);
    TurnCommand cmd = computeTurn(angleDiffRad(pose.theta, desired), mResolver, mParams.maxSpeed,
                                  mParams.minSpeed, mParams.targetTolerance, dt, ctx);
    return inPlaceTurn(cmd, mParams.maxSpeed, ctx.trackWidth);
}

// ============================================================================
// SwingMotion
// ============================================================================

SwingMotion::SwingMotion(float targetHeadingRad, SwingSide side, const SwingParams& params)
    : mTarget(targetHeadingRad), mSide(side), mParams(params), mResolver(params.direction) {}

MotionOutput SwingMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    TurnCommand cmd = computeTurn(angleDiffRad(pose.theta, mTarget), mResolver, mParams.maxSpeed,
                                  mParams.minSpeed, mParams.targetTolerance, dt, ctx);
    MotionOutput out;
    // Pivoting on a stationary side: the moving side alone sets the rotation.
    // Counter-clockwise needs the right side forward or the left side back.
    if (mSide == SwingSide::Left) {
        out.left = 0.0f;
        out.right = cmd.angular;
    } else {
        out.left = -cmd.angular;
        out.right = 0.0f;
    }
    out.done = cmd.done;
    out.remaining = std::fabs(cmd.error) * ctx.trackWidth;  // the moving wheel's arc
    return out;
}

}  // namespace FBLIB
