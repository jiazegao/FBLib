#include "FBLib/Movement_Control/RAMSETE.hpp"

#include <cmath>

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

}  // namespace FBLIB
