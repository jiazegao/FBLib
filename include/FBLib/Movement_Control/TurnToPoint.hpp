#pragma once

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

/// Heading error with an angular offset (positive = CCW)
/// @param offsetDeg  angular offset in VEX degrees
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
    float minSpeed{0.0f};
    float targetTolerance{2.0f};   // VEX degrees
};

struct TurnToPointParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{2.0f};   // VEX degrees
    float offset{0.0f};            // angular offset in VEX degrees
};

struct SwingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{2.0f};   // VEX degrees
};

}  // namespace FBLIB
