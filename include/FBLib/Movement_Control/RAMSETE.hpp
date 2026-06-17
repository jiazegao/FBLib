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
    float b{2.0f};       // convergence rate gain (higher = more aggressive)
    float zeta{0.7f};    // damping ratio (0.7 = critically damped)
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{1.0f};       // inches
    float headingTolerance{2.0f};      // VEX degrees
    float lookaheadDist{10.0f};        // pure pursuit lookahead distance
    bool useVelocityProfile{true};     // use velocity profiling for smooth motion
    float maxAccel{50.0f};             // in/s², used if velocity profile enabled
};

}  // namespace FBLIB
