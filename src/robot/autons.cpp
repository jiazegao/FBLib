#include "robot/autons.hpp"

#include <cstdint>
#include <vector>

#include "pros/rtos.hpp"
#include "robot/config.hpp"
#include "robot/mechanisms.hpp"

namespace robot {

using namespace FBLIB;

namespace {

// MCL ignores sensor rays that cross these lines during the left-side
// routes, so readings only come from walls on the robot's side
const std::vector<MclTracking::LineObstacle> leftAutonObstacles = {
    {{-72.0f, 72.0f}, {32.0f, -32.0f}},
    {{32.0f, 32.0f}, {-72.0f, -72.0f}},
    {{32.0f, -32.0f}, {32.0f, -72.0f}},
    {{32.0f, 32.0f}, {32.0f, 72.0f}},
};

// A preview only needs the path, so it skips the waits
void wait(uint32_t ms) {
    if (!chassis.isDryRun()) pros::delay(ms);
}

/// Wait until `ms` after `start` (a pros::millis() time).
void waitUntil(uint32_t start, uint32_t ms) {
    if (chassis.isDryRun()) return;
    const uint32_t elapsed = pros::millis() - start;
    if (elapsed < ms) pros::delay(ms - elapsed);
}

}  // namespace

// ============================================================================
// Localization
// ============================================================================

void initLocalization() {
    chassis.mcl().setFieldMap(fieldMap);
    for (const auto& base : rclBlockers) {
        chassis.rcl().addCircleObstacle(base.x, base.y, base.radius);
    }
}

void startLocalization(const Pose& start, const std::vector<MclTracking::LineObstacle>* mclObstacles,
                       int wallSensor) {
    chassis.setPose(start);
    if (chassis.isDryRun()) return;  // the preview simulates from `start`
    if (wallSensor >= 0) resetFromSensor(wallSensor);
    chassis.mcl().setObstacles(mclObstacles);
    chassis.mcl().startTracking();
}

bool resetFromSensor(int slot) {
    if (chassis.isDryRun()) return false;  // real sensors, simulated pose
    const Pose pose = chassis.getPose();
    chassis.rcl().updateSensorPoses(pose);
    const auto [axis, value] = chassis.rcl().getBotCoordFromSensor(slot);
    // setPosition() also moves MCL's particles and the RCL estimate
    if (axis == CoordType::X) {
        chassis.setPosition(value, pose.y);
    } else if (axis == CoordType::Y) {
        chassis.setPosition(pose.x, value);
    } else {
        return false;
    }
    return true;
}

void startAutonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    setLiftHold(true);
    homeEffector();
}

// ============================================================================
// Routes
// ============================================================================
//
// Ported from the LemLib version. There, every exit range was 0, so each
// motion ran for its whole timeout (unless minSpeed let it end on passing the
// target), and an async call waited for the previous motion before starting.
// FBLib motions end when they arrive, so where the robot was meant to stay put
// for the rest of a motion (intaking, waiting for a mechanism), waitUntil()
// keeps that time.

void left30() {
    // Facing the toggle side wall from (60, 0); the front-left sensor fixes X
    startLocalization({60.0f, 0.0f, vexToStdRad(90.0f)}, &leftAutonObstacles, FRONT_L);
    startFrontIntake();

    // Toggle: back off and ram it, twice. Each ram is stopped by the wall and
    // runs to its timeout.
    chassis.moveDistance(-5.0f, 300, {.minSpeed = 60.0f});
    chassis.moveDistance(15.0f, 600, {.minSpeed = 100.0f});
    chassis.moveDistance(-5.0f, 300, {.minSpeed = 60.0f});
    chassis.moveDistance(15.0f, 600, {.minSpeed = 100.0f});

    // Get the cup
    chassis.moveDistance(-10.0f, 800, {.minSpeed = 1.0f});
    chassis.turnToHeading(270.0f, 800, {.maxSpeed = 100.0f});
    chassis.moveToPoint(13.5f, -2.0f, 500, {.minSpeed = 100.0f});
    const uint32_t onCup = pros::millis();
    chassis.moveToPoint(13.5f, -2.0f, 1000, {.maxSpeed = 60.0f}, true);
    startFrontIntake();
    startEffectorIntake();
    waitUntil(onCup, 1000);  // keep intaking until the approach's time is up

    // Score pin + cup on the neutral base, backing in
    chassis.turnToPoint(47.5f, -23.5f, 500, {.offset = 180.0f});
    chassis.moveToPoint(47.5f, -23.5f, 1600, {.forwards = false, .maxSpeed = 75.0f}, true);
    wait(500);
    setEffector(RIGHT_ANGLE);
    wait(300);
    setLift(12.0f);
    wait(600);
    scoreCup(2.0f, 15.0f);
    wait(800);
    setEffector(HIGH_ANGLE);

    // Grab pin
    const uint32_t toPin = pros::millis();
    chassis.moveToPoint(40.0f, 8.0f, 1200, {}, true);
    resetLift();
    startEffectorIntake();
    waitUntil(toPin, 1200);
    const uint32_t creep = pros::millis();
    chassis.moveToPoint(40.0f, 8.0f, 800, {.maxSpeed = 40.0f}, true);
    reverseFrontIntake();
    waitUntil(creep, 800);

    const uint32_t turn = pros::millis();
    chassis.turnToPoint(24.0f, 23.5f, 500, {.offset = 180.0f}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    waitUntil(turn, 500);  // give the effector the turn's time to swing up
    chassis.moveToPoint(23.0f, 25.0f, 300, {.forwards = false});
    const uint32_t onPin = pros::millis();
    chassis.moveToPoint(23.0f, 25.0f, 500, {.forwards = false, .maxSpeed = 40.0f});
    waitUntil(onPin, 500);

    setEffector(EFFECTOR_ANGLES[LOW_ANGLE] - 6.0f);
    wait(400);
    setEffector(RIGHT_ANGLE);
    wait(300);

    // Score again. (The LemLib version also set the drive motors to 10/-120
    // here, but its approach was still running and overwrote them: the robot
    // never pivoted. Left out, so the robot keeps doing what it was tested
    // doing.)
    chassis.turnToPoint(45.0f, -26.0f, 700, {.offset = 180.0f});
    chassis.moveToPoint(45.0f, -26.0f, 2700, {.forwards = false, .maxSpeed = 80.0f}, true);
    setLift(15.0f);
    wait(1500);
    scoreCup(4.0f, 4.0f);
}

}  // namespace robot
