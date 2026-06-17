#pragma once

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Arc utility functions
// ============================================================================

/// Compute the center of curvature for an arc.
/// Positive radius = center to the left (CCW arc), negative = right (CW).
Pose computeArcCenter(const Pose& current, float radius);

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
    bool forwards{true};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{1.0f};
    float lead{0.0f};
};

}  // namespace FBLIB
