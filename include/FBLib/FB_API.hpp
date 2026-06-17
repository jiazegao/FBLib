#pragma once

// ============================================================================
// FBLib — Single-include public API
// ============================================================================
//
// Include this header to get the entire FBLib library.
//
// Example:
//   #include "FBLib/FB_API.hpp"
//   using namespace FBLIB;
//
//   Drivetrain dt(&leftMotors, &rightMotors, 10.4f, 3.25f);
//   OdomSensors sensors = { ... };
//   Chassis chassis(dt, sensors);
//   chassis.calibrate();
//   chassis.setPose({0, 0, 0});
//   chassis.moveToPoint(24, 0, 2000);
// ============================================================================

// — Foundation —
#include "FBLib/Util/Util.hpp"
#include "FBLib/Util/Pid.hpp"
#include "FBLib/Util/FastTrig.hpp"
#include "FBLib/Util/ScaledIMU.hpp"

// — Tracking —
#include "FBLib/Tracking/Odom_Tracking.hpp"
#include "FBLib/Tracking/RCL_Tracking.hpp"
#include "FBLib/Tracking/MCL_Tracking.hpp"

// — Movement Control —
#include "FBLib/Movement_Control/MoveToPoint.hpp"
#include "FBLib/Movement_Control/TurnToPoint.hpp"
#include "FBLib/Movement_Control/Arc.hpp"
#include "FBLib/Movement_Control/Boomerang.hpp"
#include "FBLib/Movement_Control/RAMSETE.hpp"
#include "FBLib/Movement_Control/Velocity_Profiles.hpp"

// — Chassis (integration) —
#include "FBLib/Chassis.hpp"

// — Auton Selector —
#include "FBLib/Auton_Selector/GUI.hpp"
#include "FBLib/Auton_Selector/Path_Selector.hpp"
