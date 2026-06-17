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

}  // namespace FBLIB
