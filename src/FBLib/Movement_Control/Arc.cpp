#include "FBLib/Movement_Control/Arc.hpp"

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
    float perpX, perpY;
    if (radius >= 0.0f) {
        perpX =  std::sin(current.theta);
        perpY = -std::cos(current.theta);
    } else {
        perpX = -std::sin(current.theta);
        perpY =  std::cos(current.theta);
    }
    float absRadius = std::fabs(radius);
    return {current.x + perpX * absRadius, current.y + perpY * absRadius, 0.0f};
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
    float chordBearing = bearingToPoint(current, targetX, targetY);
    float angle = arcAngle(current, targetX, targetY, radius);
    float sign = (radius >= 0.0f) ? 1.0f : -1.0f;
    return wrapRad(chordBearing + sign * (HALF_PI - angle * 0.5f));
}

}  // namespace FBLIB
