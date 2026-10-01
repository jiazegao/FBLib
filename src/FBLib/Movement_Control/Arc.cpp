#include "FBLib/Movement_Control/Arc.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Arc — circular arc movement utilities
// ============================================================================

/// Compute the center of curvature for an arc.
/// Positive radius = center to the left of the robot (CCW arc).
/// Negative radius = center to the right of the robot (CW arc).
Pose computeArcCenter(const Pose& current, float radius) {
    // The robot's left is its heading rotated +90 deg: (-sin, cos).
    float side = (radius >= 0.0f) ? 1.0f : -1.0f;
    float absRadius = std::fabs(radius);
    return {current.x - side * std::sin(current.theta) * absRadius,
            current.y + side * std::cos(current.theta) * absRadius, 0.0f};
}

Pose arcCenterThroughPoints(float x0, float y0, float x1, float y1, float radius) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float chord = std::sqrt(dx * dx + dy * dy);
    if (chord < 1e-6f) return {x0, y0, 0.0f};

    // Travelling counter-clockwise along the shorter arc keeps the center on
    // the left of the chord direction (clockwise: on the right).
    float half = 0.5f * chord;
    float r = std::max(std::fabs(radius), half);
    float h = std::sqrt(std::max(r * r - half * half, 0.0f));
    float side = (radius >= 0.0f) ? 1.0f : -1.0f;
    float ux = dx / chord;
    float uy = dy / chord;
    return {0.5f * (x0 + x1) - side * h * uy, 0.5f * (y0 + y1) + side * h * ux, 0.0f};
}

/// Compute the chord distance from current pose to a target point.
float chordDistance(const Pose& current, float targetX, float targetY) {
    return distanceToPoint(current, targetX, targetY);
}

/// Compute the arc angle (radians) for an arc from current to target
/// with the given radius.
float arcAngle(const Pose& current, float targetX, float targetY,
               float radius) {
    float chord = chordDistance(current, targetX, targetY);
    float absRadius = std::fabs(radius);
    if (absRadius < 1e-6f) return 0.0f;
    float halfAngle = std::asin(std::min(chord / (2.0f * absRadius), 1.0f));
    return 2.0f * halfAngle;
}

/// Compute the tangent heading at the end of an arc from current to target.
float arcEndHeading(const Pose& current, float targetX, float targetY,
                    float radius) {
    // The tangents at the two ends of an arc sit half the arc angle either
    // side of the chord: start = chord - angle/2, end = chord + angle/2
    // (signs flipped for a clockwise arc).
    float chordBearing = bearingToPoint(current, targetX, targetY);
    float angle = arcAngle(current, targetX, targetY, radius);
    float sign = (radius >= 0.0f) ? 1.0f : -1.0f;
    return wrapRad(chordBearing + sign * angle * 0.5f);
}

// ============================================================================
// ArcMotion
// ============================================================================

namespace {
/// Distance over which the robot steers back onto the circle when it has
/// drifted off it radially (larger = gentler correction).
constexpr float ARC_CORRECTION_DIST = 8.0f;  // inches
}  // namespace

ArcMotion::ArcMotion(float x, float y, float radius, const ArcParams& params)
    : mTargetX(x), mTargetY(y), mRadius(radius), mParams(params) {}

void ArcMotion::start(const Pose& pose, const MotionContext& ctx) {
    float chord = distanceToPoint(pose, mTargetX, mTargetY);
    if (chord < 1e-3f) {
        mAlreadyThere = true;
        return;
    }
    if (std::fabs(mRadius) < 1e-3f) {
        MoveToPointParams line;
        line.forwards = mParams.forwards;
        line.maxSpeed = mParams.maxSpeed;
        line.minSpeed = mParams.minSpeed;
        line.targetTolerance = mParams.targetTolerance;
        mLine = std::make_unique<MoveToPointMotion>(mTargetX, mTargetY, line);
        mLine->start(pose, ctx);
        return;
    }

    Pose center = arcCenterThroughPoints(pose.x, pose.y, mTargetX, mTargetY, mRadius);
    mCenterX = center.x;
    mCenterY = center.y;
    mR = std::max(std::fabs(mRadius), 0.5f * chord);
    mSign = (mRadius > 0.0f) ? 1.0f : -1.0f;

    float startAngle = std::atan2(pose.y - mCenterY, pose.x - mCenterX);
    float targetAngle = std::atan2(mTargetY - mCenterY, mTargetX - mCenterX);
    mSweep = wrapRadPositive(mSign * (targetAngle - startAngle));
    mProgress = 0.0f;
    mPrevAngle = startAngle;
}

MotionOutput ArcMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    if (mAlreadyThere) {
        MotionOutput out;
        out.done = true;
        return out;
    }
    if (mLine) return mLine->update(pose, dt, ctx);

    // Where the robot is around the circle, and how much arc is left
    float rx = pose.x - mCenterX;
    float ry = pose.y - mCenterY;
    float rho = std::sqrt(rx * rx + ry * ry);
    float angle = std::atan2(ry, rx);
    mProgress += mSign * angleDiffRad(mPrevAngle, angle);
    mPrevAngle = angle;
    float remaining = (mSweep - mProgress) * mR;  // signed; negative once past the target

    // Steer along the tangent, turning back toward the circle if off it
    float tangent = angle + mSign * HALF_PI;
    float radialError = rho - mR;  // + = outside the circle
    float desired = tangent + mSign * std::atan2(radialError, ARC_CORRECTION_DIST);
    float travelHeading = mParams.forwards ? pose.theta : wrapRad(pose.theta + PI);
    float angularError = angleDiffRad(travelHeading, desired);

    // Speed along the direction of travel (negative = backing up after an overshoot)
    float travel = clamp(ctx.lateralPID.update(remaining, dt), -mParams.maxSpeed, mParams.maxSpeed);
    travel = applyMinSpeed(travel, mParams.minSpeed);
    travel *= std::max(std::cos(angularError), 0.0f);  // turn first if facing away from the arc

    // Following a circle of radius r at speed v needs (vR - vL) = v * W / r;
    // feed that forward so the PID only corrects residual heading error.
    float angular = clamp(ctx.angularPID.update(angularError, dt), -mParams.maxSpeed, mParams.maxSpeed);
    angular += mSign * travel * ctx.trackWidth / (2.0f * mR);

    float lateral = mParams.forwards ? travel : -travel;
    MotionOutput out = mixDrive(lateral, angular, mParams.maxSpeed);
    // Arc still to drive plus the turn still needed (e.g. lining up at the start)
    out.remaining = std::fabs(remaining) + std::fabs(angularError) * ctx.trackWidth * 0.5f;

    bool settled = std::fabs(remaining) < mParams.targetTolerance;
    bool passed = mParams.minSpeed > 0.0f && remaining < 0.0f;
    out.done = settled || passed;
    return out;
}

}  // namespace FBLIB
