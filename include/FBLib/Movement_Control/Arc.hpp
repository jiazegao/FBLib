#pragma once

#include <memory>

#include "FBLib/Movement_Control/Motion.hpp"
#include "FBLib/Movement_Control/MoveToPoint.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Arc utility functions
// ============================================================================

/// Compute the center of curvature for an arc tangent to the current heading.
/// Positive radius = center to the left (CCW arc), negative = right (CW).
Pose computeArcCenter(const Pose& current, float radius);

/// Center of the circle of radius |radius| through (x0,y0) and (x1,y1) whose
/// shorter arc runs counter-clockwise from the first point to the second for
/// a positive radius (clockwise for negative). A radius shorter than half the
/// chord is stretched to exactly half the chord (a semicircle).
Pose arcCenterThroughPoints(float x0, float y0, float x1, float y1, float radius);

/// Chord distance from current pose to a target point (inches)
float chordDistance(const Pose& current, float targetX, float targetY);

/// Arc angle in radians for an arc from current to target with given radius
float arcAngle(const Pose& current, float targetX, float targetY, float radius);

/// Tangent heading at the end of an arc from current to target
float arcEndHeading(const Pose& current, float targetX, float targetY,
                    float radius);

// ============================================================================
// Parameter structs
// ============================================================================

struct ArcParams {
    bool forwards{true};           // false drives the arc backwards
    float maxSpeed{127.0f};
    float minSpeed{0.0f};          // > 0: never slow below this; exits on passing the target (chaining)
    float targetTolerance{1.0f};   // inches of arc length
    float lead{0.0f};              // no effect (reserved)
};

// ============================================================================
// ArcMotion — follow a circular arc to a target point
// ============================================================================
//
// The circle is fixed at start(): radius |radius| through the start position
// and the target (the shorter arc; positive radius = counter-clockwise). The
// robot steers along the circle's tangent, corrected toward the circle when
// it drifts off, and drives on the signed arc length still to go. A radius of
// ~0 drives a straight line to the target instead.
// ============================================================================

class ArcMotion : public Motion {
public:
    ArcMotion(float x, float y, float radius, const ArcParams& params);

    void start(const Pose& pose, const MotionContext& ctx) override;
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTargetX;
    float mTargetY;
    float mRadius;
    ArcParams mParams;

    std::unique_ptr<MoveToPointMotion> mLine;  // radius ~0: straight line
    bool mAlreadyThere{false};
    float mCenterX{0.0f}, mCenterY{0.0f};
    float mR{0.0f};          // circle radius actually used
    float mSign{1.0f};       // +1 counter-clockwise, -1 clockwise
    float mSweep{0.0f};      // total arc angle to travel (rad)
    float mProgress{0.0f};   // arc angle travelled so far (rad, signed)
    float mPrevAngle{0.0f};  // robot's angle around the center on the last tick
};

}  // namespace FBLIB
