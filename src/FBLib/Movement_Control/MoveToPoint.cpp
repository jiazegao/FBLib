#include "FBLib/Movement_Control/MoveToPoint.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// MoveDistanceMotion
// ============================================================================

MoveDistanceMotion::MoveDistanceMotion(float distance, const MoveDistanceParams& params)
    : mDistance(distance), mParams(params) {}

void MoveDistanceMotion::start(const Pose& pose, const MotionContext& ctx) {
    (void)ctx;
    // Fix the target ONCE. Re-anchoring it to the live pose every tick keeps
    // the error pinned at the requested distance and the motion never ends.
    float travel = mDistance * (mParams.forwards ? 1.0f : -1.0f);
    mTravelSign = (travel >= 0.0f) ? 1.0f : -1.0f;
    mHeading = pose.theta;
    mTargetX = pose.x + travel * std::cos(mHeading);
    mTargetY = pose.y + travel * std::sin(mHeading);
}

MotionOutput MoveDistanceMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    // Signed distance still to go along the held heading (+ = ahead of the robot)
    float remaining = (mTargetX - pose.x) * std::cos(mHeading) +
                      (mTargetY - pose.y) * std::sin(mHeading);

    float lateral = clamp(ctx.lateralPID.update(remaining, dt), -mParams.maxSpeed, mParams.maxSpeed);
    lateral = applyMinSpeed(lateral, mParams.minSpeed);
    float angular = clamp(ctx.angularPID.update(angleDiffRad(pose.theta, mHeading), dt),
                          -mParams.maxSpeed, mParams.maxSpeed);

    MotionOutput out = mixDrive(lateral, angular, mParams.maxSpeed);
    out.remaining = std::fabs(remaining);

    bool settled = std::fabs(remaining) < mParams.targetTolerance;
    bool earlyExit = mParams.earlyExitRange > 0.0f && std::fabs(remaining) < mParams.earlyExitRange;
    // With a minimum speed the robot can't stop on the target; end the
    // motion once it drives past it so the next motion takes over.
    bool passed = mParams.minSpeed > 0.0f && remaining * mTravelSign < 0.0f;
    out.done = settled || earlyExit || passed;
    return out;
}

// ============================================================================
// MoveToPointMotion
// ============================================================================

MoveToPointMotion::MoveToPointMotion(float x, float y, const MoveToPointParams& params)
    : mTargetX(x), mTargetY(y), mParams(params) {}

namespace {
/// Inside MOTION_CLOSE_RANGE the heading is only locked once the robot points
/// within this of the target; until then it turns in place toward it.
constexpr float AIM_TOLERANCE = 10.0f * DEG_TO_RAD;
}  // namespace

MotionOutput MoveToPointMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    float dx = mTargetX - pose.x;
    float dy = mTargetY - pose.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    float bearing = std::atan2(dy, dx);

    // Driving backwards, the robot's "front" is its back.
    float travelHeading = mParams.forwards ? pose.theta : wrapRad(pose.theta + PI);
    float angularError = angleDiffRad(travelHeading, bearing);
    // Signed projection onto the robot's forward axis: negative once the
    // target is behind the robot, so an overshoot backs up.
    float lateralError = dist * std::cos(angleDiffRad(pose.theta, bearing));

    bool close = dist < MOTION_CLOSE_RANGE;
    if (!mLocked && close && std::fabs(angularError) < AIM_TOLERANCE) mLocked = true;

    float lateral = clamp(ctx.lateralPID.update(lateralError, dt), -mParams.maxSpeed, mParams.maxSpeed);
    float angular = clamp(ctx.angularPID.update(angularError, dt), -mParams.maxSpeed, mParams.maxSpeed);
    if (mLocked) {
        angular = 0.0f;  // stop steering at the point (see MOTION_CLOSE_RANGE)
    } else if (close) {
        // Close but not aimed (e.g. started a few inches beside the target):
        // turn in place first. Rotating doesn't move the bearing, so this is
        // stable; driving while steering this close to the point is not.
        lateral = 0.0f;
    } else if (mParams.forwards) {
        lateral = std::max(lateral, 0.0f);  // turn toward a target behind instead of reversing
    } else {
        lateral = std::min(lateral, 0.0f);
    }
    lateral = applyMinSpeed(lateral, mParams.minSpeed);

    MotionOutput out = mixDrive(lateral, angular, mParams.maxSpeed);
    // Remaining wheel travel: distance plus the turn still needed
    out.remaining = (mLocked ? std::fabs(lateralError) : dist) +
                    (mLocked ? 0.0f : std::fabs(angularError) * ctx.trackWidth * 0.5f);

    bool settled = dist < mParams.targetTolerance ||
                   (mLocked && std::fabs(lateralError) < mParams.targetTolerance);
    bool earlyExit = mParams.earlyExitRange > 0.0f && dist < mParams.earlyExitRange;
    float travelSign = mParams.forwards ? 1.0f : -1.0f;
    bool passed = mParams.minSpeed > 0.0f && mLocked && lateralError * travelSign < 0.0f;
    out.done = settled || earlyExit || passed;
    return out;
}

}  // namespace FBLIB
