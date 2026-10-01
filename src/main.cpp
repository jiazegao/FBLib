// ============================================================================
// Competition template — team 1239E's 2026-27 program on FBLib
// ============================================================================
//
//   include/robot/config.hpp      ports, sensors, gains, season field data
//   src/robot/mechanisms.cpp      intakes, end effector, lift, scoring macros
//   src/robot/driver.cpp          driver control (button map in driver.hpp)
//   src/robot/autons.cpp          autonomous routes + localization helpers
//
// This file only wires them into the PROS competition lifecycle.
// ============================================================================

#include "main.h"

#include "FBLib/FB_API.hpp"
#include "robot/autons.hpp"
#include "robot/config.hpp"
#include "robot/driver.hpp"
#include "robot/mechanisms.hpp"

using namespace FBLIB;
using namespace robot;

// Touchscreen selector with a path preview of the selected routine
AutonSelector selector;
PathPreview preview;

/// Runs once on startup, robot disabled. The lift must be all the way down.
void initialize() {
    chassis.calibrate();  // IMU, ~2 s
    initMechanisms();
    initLocalization();

    selector.registerAuton("Left 30", left30);
    selector.init();
    preview.init();
    selector.enableDryRun(chassis, preview);
}

/// Runs while disabled (before and between matches). The selector screen
/// handles itself.
void disabled() {}

/// Runs once when the field connects, before autonomous.
void competition_initialize() {}

void autonomous() {
    startAutonomous();
    selector.runSelected();
}

void opcontrol() {
    startDriverControl();
    while (true) {
        driverControlStep();
        pros::delay(20);
    }
}
