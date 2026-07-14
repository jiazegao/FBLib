#pragma once

#include <vector>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// RAMSETE utility functions
// ============================================================================

/// Find the closest point index on a path to a given pose
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose);

/// Find the lookahead point index on a path (first point beyond lookaheadDist)
int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist);

// ============================================================================
// Parameter structs
// ============================================================================

struct RAMSETEParams {
    // b and zeta use the conventional (meter-based) RAMSETE parameterization —
    // the controller scales b to inches internally, so the textbook defaults
    // b≈2.0, zeta∈(0,1) behave sensibly without retuning.
    float b{2.0f};       // aggressiveness (higher = tighter tracking), b > 0
    float zeta{0.7f};    // damping ratio, 0 < zeta < 1
    float maxSpeed{127.0f};            // peak output, motor units (0–127)
    float minSpeed{0.0f};
    float targetTolerance{1.0f};       // inches
    float headingTolerance{2.0f};      // VEX degrees
    float lookaheadDist{10.0f};        // pure pursuit lookahead distance (inches)
    bool useVelocityProfile{true};     // profile v_d for accel/decel along path
    float maxAccel{50.0f};             // in/s², used when useVelocityProfile
};

}  // namespace FBLIB
