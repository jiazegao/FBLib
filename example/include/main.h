#pragma once

#include <array>
#include <vector>

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include "FBLib/FB_API.hpp"

// ============================================================================
// 1239E Flashbang — 2025-26 "High Stakes" Robot Configuration
// ============================================================================
//
// Drive base:     4-motor tank (2 left, 2 right), 3.25" wheels, 10.4" track
// Tracking:       2 ADI dead wheels (vertical + horizontal) + V5 IMU
// Distance:       4 V5 distance sensors (forward, left, back, right)
// Controller:     Tank drive with exponential throttle curve
// ============================================================================

// — Drive motors (smart ports 1-4) —
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// — IMU (smart port 5) —
extern pros::Imu imu;

// — ADI encoders for tracking wheels (ADI ports A, B) —
extern pros::adi::Encoder vertEnc;
extern pros::adi::Encoder horizEnc;

// — Tracking wheels (2.75" omnis with direct gearing) —
extern FBLIB::TrackingWheel vertWheel;
extern FBLIB::TrackingWheel horizWheel;

// — Distance sensors (smart ports 6-9) —
extern pros::Distance distFront;
extern pros::Distance distLeft;
extern pros::Distance distBack;
extern pros::Distance distRight;

// — Chassis (the top-level robot object) —
extern FBLIB::Chassis chassis;

// — Auton Selector —
extern FBLIB::AutonSelector selector;
extern FBLIB::PathPreview pathPreview;

// — Controller —
extern pros::Controller master;

// ============================================================================
// Autonomous routine declarations (one per strategy)
// ============================================================================

void autonSkills();
void autonRedOffensive();
void autonRedDefensive();
void autonBlueOffensive();
void autonBlueDefensive();
