#pragma once

// ============================================================================
// Autonomous routines and their localization helpers
// ============================================================================

#include <vector>

#include "FBLib/FB_API.hpp"

namespace robot {

/// Give MCL the season's field elements and RCL the goal bases that block
/// wall readings. Call once from initialize().
void initLocalization();

/// Route start: set the starting pose, square one coordinate up on a wall
/// with `wallSensor` (a DistanceSlot, or -1 for none), then start MCL with
/// `mclObstacles` (rays crossing them are ignored; must outlive the route).
/// In a preview only the pose is set.
void startLocalization(const FBLIB::Pose& start,
                       const std::vector<FBLIB::MclTracking::LineObstacle>* mclObstacles,
                       int wallSensor);

/// Correct X or Y (whichever the sensor's wall gives) from one distance
/// sensor. Returns false, changing nothing, without a usable reading.
bool resetFromSensor(int slot);

/// Call at the start of autonomous(), before the route: brakes on hold, the
/// lift holding its target, and the effector homing (~1.2 s, while the route
/// starts driving).
void startAutonomous();

// ---- Routes ----

/// Left side: toggle twice, score a cup and a pin on the neutral base at
/// (47.5, -23.5), grab another cup and score it there too.
void left30();

}  // namespace robot
