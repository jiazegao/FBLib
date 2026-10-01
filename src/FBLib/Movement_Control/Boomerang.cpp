#include "FBLib/Movement_Control/Boomerang.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// BoomerangMotion — carrot-point guidance with lead decay
// ============================================================================
//
// Reference: LemLib boomerang controller (clean-room implementation).
// ============================================================================

namespace {
constexpr float BOOMERANG_MAX_LEAD = 24.0f;      // inches, cap on the carrot offset
constexpr float BOOMERANG_BLEND_RANGE = 12.0f;   // inches, start blending to the final heading
}  // namespace

BoomerangMotion::BoomerangMotion(float x, float y, float thetaRad, const BoomerangParams& params)
    : mTargetX(x), mTargetY(y), mTargetTheta(thetaRad), mParams(params) {}

MotionOutput BoomerangMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    float dx = mTargetX - pose.x;
    float dy = mTargetY - pose.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (!mClose && dist < MOTION_CLOSE_RANGE) mClose = true;

    // Direction of travel on arrival, and the robot's current direction of
    // travel (its back when driving backwards).
    float arrival = mParams.forwards ? mTargetTheta : wrapRad(mTargetTheta + PI);
    float travelHeading = mParams.forwards ? pose.theta : wrapRad(pose.theta + PI);
    float finalError = angleDiffRad(travelHeading, arrival);

    float angularError;
    float lateralError;
    if (mClose) {
        // Hold the final heading and drive on the signed distance.
        angularError = finalError;
        lateralError = dist * std::cos(angleDiffRad(pose.theta, std::atan2(dy, dx)));
    } else {
        // Carrot: behind the target along the arrival direction. Offsetting it
        // along the FINAL heading (not the bearing to the target) is what
        // bends the approach so the robot arrives facing the right way.
        float offset = std::min(mParams.lead * dist, BOOMERANG_MAX_LEAD);
        float far = (mParams.lead > 1e-6f) ? std::min(dist / (mParams.lead * 12.0f), 1.0f) : 1.0f;
        offset *= 1.0f - far * mParams.leadDecay;
        float carrotX = mTargetX - offset * std::cos(arrival);
        float carrotY = mTargetY - offset * std::sin(arrival);
        float carrotBearing = std::atan2(carrotY - pose.y, carrotX - pose.x);
        float carrotDist = std::sqrt((carrotX - pose.x) * (carrotX - pose.x) +
                                     (carrotY - pose.y) * (carrotY - pose.y));

        // Steer at the carrot, blending to the final heading as the target
        // nears. Interpolate along the shortest arc: a plain average of two
        // angles either side of +-pi points the wrong way. Only blend once
        // the carrot is in front: with it behind (no forward drive), turning
        // toward the carrot and toward the final heading can cancel out and
        // leave the robot sitting still.
        float carrotError = angleDiffRad(travelHeading, carrotBearing);
        float weight = clamp((dist - MOTION_CLOSE_RANGE) / (BOOMERANG_BLEND_RANGE - MOTION_CLOSE_RANGE),
                             0.0f, 1.0f);
        if (std::fabs(carrotError) > HALF_PI) weight = 1.0f;
        angularError = wrapRad(carrotError + (1.0f - weight) * angleDiffRad(carrotError, finalError));

        // Distance to the carrot, negative when it is behind the robot.
        float facing = std::cos(angleDiffRad(pose.theta, carrotBearing));
        lateralError = (facing >= 0.0f) ? carrotDist : -carrotDist;
    }

    float lateral = clamp(ctx.lateralPID.update(lateralError, dt), -mParams.maxSpeed, mParams.maxSpeed);
    float angular = clamp(ctx.angularPID.update(angularError, dt), -mParams.maxSpeed, mParams.maxSpeed);
    if (!mClose) {
        // Far away, turn toward a carrot on the wrong side instead of reversing.
        lateral = mParams.forwards ? std::max(lateral, 0.0f) : std::min(lateral, 0.0f);
    }
    lateral = applyMinSpeed(lateral, mParams.minSpeed);

    MotionOutput out = mixDrive(lateral, angular, mParams.maxSpeed);
    // Distance plus the turn still needed toward the current steering target
    out.remaining = dist + std::fabs(angularError) * ctx.trackWidth * 0.5f;

    bool positionOk = dist < mParams.targetTolerance ||
                      (mClose && std::fabs(lateralError) < mParams.targetTolerance);
    bool headingOk = std::fabs(angleDiffRad(pose.theta, mTargetTheta)) < degToRad(mParams.headingTolerance);
    float travelSign = mParams.forwards ? 1.0f : -1.0f;
    bool passed = mParams.minSpeed > 0.0f && mClose && lateralError * travelSign < 0.0f;
    out.done = (positionOk && headingOk) || passed;
    return out;
}

}  // namespace FBLIB
