#include "FBLib/Movement_Control/RAMSETE.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Movement_Control/Velocity_Profiles.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// RAMSETE — nonlinear SE(2) trajectory tracking path utilities
// ============================================================================

/// Find the closest point index on a path to a given pose.
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose) {
    if (path.empty()) return -1;

    int bestIdx = 0;
    float bestDist = distanceToPoint(pose, path[0].x, path[0].y);

    for (size_t i = 1; i < path.size(); i++) {
        float d = distanceToPoint(pose, path[i].x, path[i].y);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = static_cast<int>(i);
        }
    }
    return bestIdx;
}

/// Find the lookahead point on a path (first point beyond lookaheadDist).
int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist) {
    if (path.empty()) return -1;
    int closest = closestPathIndex(path, pose);
    for (size_t i = static_cast<size_t>(closest); i < path.size(); i++) {
        if (distanceToPoint(pose, path[i].x, path[i].y) >= lookaheadDist) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(path.size()) - 1;
}

// ============================================================================
// RamseteMotion
// ============================================================================
//
// Standard unicycle tracking law (WPILib / De Luca convention):
//   k  = 2ζ·√(ω_d² + b·v_d²)
//   v  = v_d·cos(e_θ) + k·e_x                          [in/s]
//   ω  = ω_d + b·v_d·sinc(e_θ)·e_y + k·e_θ             [rad/s]
//
// Everything is in real units: v/v_d in inches/second, ω/ω_d in
// radians/second. v_d comes from a velocity profile over the path
// (accel/decel), NOT a constant. `b` is taken in the conventional
// meter-based parameterization (b≈2.0, ζ≈0.7) and scaled to inches.
// ============================================================================

RamseteMotion::RamseteMotion(const std::vector<Pose>& path, const RAMSETEParams& params)
    : mPath(path), mParams(params) {}

void RamseteMotion::start(const Pose& pose, const MotionContext& ctx) {
    const size_t n = mPath.size();
    mCumLen.assign(n, 0.0f);
    for (size_t i = 1; i < n; ++i) {
        mCumLen[i] = mCumLen[i - 1] + distance(mPath[i - 1].x, mPath[i - 1].y, mPath[i].x, mPath[i].y);
    }
    mTotalLen = n > 0 ? mCumLen.back() : 0.0f;
    mCruise = ctx.maxSpeedInPerSec * clamp(mParams.maxSpeed, 0.0f, 127.0f) / 127.0f;
    mClosest = std::max(closestPathIndex(mPath, pose), 0);

    // Direction of travel at the end: last segment with non-zero length.
    mEndTangent = n > 0 ? mPath.back().theta : 0.0f;
    for (size_t i = n; i >= 2; --i) {
        float dx = mPath[i - 1].x - mPath[i - 2].x;
        float dy = mPath[i - 1].y - mPath[i - 2].y;
        if (dx * dx + dy * dy > 1e-6f) {
            mEndTangent = std::atan2(dy, dx);
            break;
        }
    }
}

float RamseteMotion::progressAlongPath(const Pose& pose) const {
    // Project onto the segment after the closest point (or before it, if the
    // robot hasn't reached that point yet) for progress that moves smoothly
    // between points instead of jumping at each one.
    const int n = static_cast<int>(mPath.size());
    auto project = [&](int a) {
        const Pose& p0 = mPath[a];
        const Pose& p1 = mPath[a + 1];
        float sx = p1.x - p0.x, sy = p1.y - p0.y;
        float len2 = sx * sx + sy * sy;
        if (len2 < 1e-9f) return mCumLen[a];
        float t = ((pose.x - p0.x) * sx + (pose.y - p0.y) * sy) / len2;
        return mCumLen[a] + clamp(t, 0.0f, 1.0f) * std::sqrt(len2);
    };
    if (mClosest + 1 < n) {
        float ahead = project(mClosest);
        if (ahead > mCumLen[mClosest] || mClosest == 0) return ahead;
    }
    if (mClosest > 0) return project(mClosest - 1);
    return mCumLen[mClosest];
}

MotionOutput RamseteMotion::update(const Pose& pose, float dt, const MotionContext& ctx) {
    (void)dt;
    MotionOutput out;
    const int n = static_cast<int>(mPath.size());
    if (n < 2) {
        out.done = true;
        return out;
    }

    // — Progress: the closest point only moves forward, searched within a
    //   lookahead-sized window, so a path that passes near itself can't make
    //   progress jump ahead or back. —
    float bestDist = distanceToPoint(pose, mPath[mClosest].x, mPath[mClosest].y);
    for (int i = mClosest + 1; i < n; ++i) {
        float d = distanceToPoint(pose, mPath[i].x, mPath[i].y);
        if (d < bestDist) {
            bestDist = d;
            mClosest = i;
        }
        if (mCumLen[i] - mCumLen[mClosest] > mParams.lookaheadDist) break;
    }
    float progress = progressAlongPath(pose);

    // — Reference pose: lookaheadDist of arc length ahead of the progress —
    int targetIdx = mClosest;
    while (targetIdx < n - 1 && mCumLen[targetIdx] - progress < mParams.lookaheadDist) ++targetIdx;
    const Pose& target = mPath[targetIdx];

    // — Desired forward speed v_d (in/s). The profile starts and ends at a
    //   small non-zero speed: at exactly zero the tracking gain k is zero too
    //   and the robot would never leave the start of the path. —
    float minSpeedInPerSec = ctx.maxSpeedInPerSec * clamp(mParams.minSpeed, 0.0f, 127.0f) / 127.0f;
    float edgeSpeed = std::min(mCruise, std::max(0.1f * mCruise, minSpeedInPerSec));
    float v_d = mParams.useVelocityProfile
                    ? trapezoidalVelocityAt(progress, mTotalLen, mCruise, mParams.maxAccel, edgeSpeed)
                    : mCruise;

    // — Reference angular velocity ω_d = v_d · curvature (rad/s) —
    float w_d = 0.0f;
    if (targetIdx > 0 && targetIdx < n - 1) {
        const Pose& prev = mPath[targetIdx - 1];
        const Pose& next = mPath[targetIdx + 1];
        float segDist = distanceToPoint(prev, next.x, next.y);
        if (segDist > 1e-3f) {
            w_d = v_d * angleDiffRad(prev.theta, next.theta) / segDist;
        }
    }

    // — Pose error in the robot frame —
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    float e_x = dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
    float e_y = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
    float e_theta = angleDiffRad(pose.theta, target.theta);

    // — RAMSETE gains + law (b scaled meters→inches) —
    float b = mParams.b * (INCH_TO_METER * INCH_TO_METER);
    float k = 2.0f * mParams.zeta * std::sqrt(w_d * w_d + b * v_d * v_d);
    float sinc = (std::fabs(e_theta) < 1e-4f) ? 1.0f : std::sin(e_theta) / e_theta;
    float v = v_d * std::cos(e_theta) + k * e_x;           // in/s
    float w = w_d + b * v_d * sinc * e_y + k * e_theta;    // rad/s

    // — (v, ω) → wheel speeds → motor units. CCW ω speeds up the right side. —
    float halfTrack = ctx.trackWidth * 0.5f;
    float toMotor = (ctx.maxSpeedInPerSec > 1e-3f) ? (127.0f / ctx.maxSpeedInPerSec) : 0.0f;
    float lateral = v * toMotor;
    float angular = w * halfTrack * toMotor;
    out = mixDrive(lateral, angular, mParams.maxSpeed);

    // — Completion —
    const Pose& end = mPath.back();
    float distToEnd = distanceToPoint(pose, end.x, end.y);
    float aheadOfEnd = (end.x - pose.x) * std::cos(mEndTangent) + (end.y - pose.y) * std::sin(mEndTangent);
    bool atEnd = distToEnd < mParams.targetTolerance &&
                 std::fabs(angleDiffRad(pose.theta, end.theta)) < degToRad(mParams.headingTolerance);
    // RAMSETE's feed-forward keeps pushing along the path, so once the robot
    // is past the endpoint it would drive away forever — stop there instead.
    bool passedEnd = mClosest == n - 1 && aheadOfEnd < 0.0f;
    out.done = atEnd || passedEnd;
    out.remaining = std::max(mTotalLen - progress, 0.0f) + (mClosest == n - 1 ? distToEnd : 0.0f);
    return out;
}

}  // namespace FBLIB
