#pragma once

// ============================================================================
// Robot configuration — hardware, tuning and season field data
// ============================================================================
//
// Template competition program, ported from team 1239E's 2026-27 robot
// (LemLib 0.5.6). Change the ports, offsets, gains and field data here for
// your robot; the rest of src/robot/ works from these objects.
//
// Field frame (same as LemLib): inches from the field centre. Headings passed
// to FBLib calls are VEX degrees (0 = +Y, clockwise); Pose.theta is radians
// (0 = +X, counter-clockwise), so use vexToStdRad() to build a Pose.
// Every object is `inline`: one instance shared by every file.
// ============================================================================

#include <cstdint>
#include <vector>

#include "FBLib/FB_API.hpp"
#include "pros/distance.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

namespace robot {

// ----------------------------------------------------------------------------
// Controller
// ----------------------------------------------------------------------------

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ----------------------------------------------------------------------------
// Drivetrain: 2 + 2 blue motors, 3.25" wheels at 450 rpm
// ----------------------------------------------------------------------------

inline pros::MotorGroup leftMotors({10, 9}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({-19, -17}, pros::MotorGearset::blue);

inline FBLIB::Drivetrain drivetrain(&leftMotors, &rightMotors,
                                    10.4f,   // track width (inches)
                                    3.25f,   // wheel diameter (inches)
                                    450.0f,  // wheel RPM
                                    2.0f);   // horizontal drift

// ----------------------------------------------------------------------------
// Mechanisms (driven by mechanisms.cpp)
// ----------------------------------------------------------------------------

inline pros::MotorGroup liftMotors({-7, 21}, pros::MotorGearset::blue);
inline pros::Motor frontIntakeMotor(20, pros::MotorGearset::blue);
inline pros::Motor effectorIntakeMotor(-5, pros::MotorGearset::green);
inline pros::Motor effectorRotateMotor(-4, pros::MotorGearset::green);

// ----------------------------------------------------------------------------
// Sensors
// ----------------------------------------------------------------------------

// This IMU reports 360.91° for one real turn
inline FBLIB::ScaledIMU imu(16, 360.0, 360.91);

// Distance sensors, in their MCL/RCL slot order
enum DistanceSlot { FRONT_L, FRONT_R, LEFT, BACK, RIGHT };
inline pros::Distance frontLeftDist(1);
inline pros::Distance frontRightDist(8);
inline pros::Distance leftDist(18);
inline pros::Distance backDist(2);
inline pros::Distance rightDist(3);

// FRONT_L, FRONT_R and BACK sit 9.12" up, LEFT and RIGHT 4.19": the low goal
// bases pass under the high sensors' beams (see fieldLines below)
constexpr uint8_t HIGH_SENSORS = (1u << FRONT_L) | (1u << FRONT_R) | (1u << BACK);
constexpr uint8_t LOW_SENSORS = (1u << LEFT) | (1u << RIGHT);

// No tracking wheels: the drive motor encoders measure travel
inline FBLIB::OdomSensors odomSensors = {
    .vertWheelCollection = {},
    .horizWheelCollection = {},
    .imuCollection = {&imu},
    .drivetrain = &drivetrain,
};

// ----------------------------------------------------------------------------
// Chassis
// ----------------------------------------------------------------------------

inline FBLIB::ChassisConfig chassisConfig = {
    // Converted from the LemLib gains (lateral kP 10, kD 20; angular kP 3,
    // kD 19.5): LemLib's derivative is per 10 ms tick and its angular error
    // is in degrees; FBLib's derivative is per second and its angular error
    // in radians. The controllers differ, so retune on the robot.
    .lateralGains = {10.0f, 0.0f, 0.2f},
    .angularGains = {171.9f, 0.0f, 11.17f},

    // Same values and formula as the LemLib ExpoDriveCurves. Tank drive uses
    // the throttle curve on both sticks.
    .throttleCurve = {.deadband = 15.0f, .minOutput = 20.0f, .curve = 1.05f},
    .steerCurve = {.deadband = 10.0f, .minOutput = 30.0f, .curve = 1.3f},

    .distanceSensors = {&frontLeftDist, &frontRightDist, &leftDist, &backDist, &rightDist,
                        nullptr, nullptr, nullptr},
    // Mount offsets: x forward, y left (inches from the tracking centre),
    // theta = direction the sensor points (radians, CCW from forward)
    .sensorMounts = {{
        {4.656f, 5.694f, 0.0f},                 // FRONT_L
        {4.656f, -5.694f, 0.0f},                // FRONT_R
        {-3.000f, 5.857f, FBLIB::HALF_PI},      // LEFT
        {-0.920f, 5.694f, FBLIB::PI},           // BACK
        {-3.000f, -5.857f, -FBLIB::HALF_PI},    // RIGHT
    }},

    // Each route starts MCL itself (startLocalization); RCL is only used for
    // one-off wall resets (resetFromSensor)
    .useMclTracking = false,
    .useRclTracking = false,
    .mclConfig = {.distSyncProp = 0.05f, .confidenceThreshold = 20, .minSensorRange = 10.0f},
    .rclConfig = {.angleTolerance = 15.0f, .confidenceThreshold = 60, .autoSync = false},
};

inline FBLIB::Chassis chassis(drivetrain, odomSensors, chassisConfig);

// ----------------------------------------------------------------------------
// Season field data (2026-27)
// ----------------------------------------------------------------------------

// What the distance sensors can see inside the walls, for MCL
inline const std::vector<FBLIB::MclTracking::LineObstacle> fieldLines = {
    // Match loaders, one 4" x 4" box per corner (every sensor)
    {{70.5f, 56.75f}, {66.5f, 56.75f}},     {{66.5f, 56.75f}, {66.5f, 60.75f}},
    {{66.5f, 60.75f}, {70.5f, 60.75f}},
    {{-70.5f, 56.75f}, {-66.5f, 56.75f}},   {{-66.5f, 56.75f}, {-66.5f, 60.75f}},
    {{-66.5f, 60.75f}, {-70.5f, 60.75f}},
    {{-70.5f, -56.75f}, {-66.5f, -56.75f}}, {{-66.5f, -56.75f}, {-66.5f, -60.75f}},
    {{-66.5f, -60.75f}, {-70.5f, -60.75f}},
    {{70.5f, -56.75f}, {66.5f, -56.75f}},   {{66.5f, -56.75f}, {66.5f, -60.75f}},
    {{66.5f, -60.75f}, {70.5f, -60.75f}},

    // Centre base, low part (low sensors)
    {{2.806f, 2.806f}, {2.806f, -2.806f}, LOW_SENSORS},
    {{2.806f, -2.806f}, {-2.806f, -2.806f}, LOW_SENSORS},
    {{-2.806f, -2.806f}, {-2.806f, 2.806f}, LOW_SENSORS},
    {{-2.806f, 2.806f}, {2.806f, 2.806f}, LOW_SENSORS},

    // Neutral bases, 5" squares (low sensors)
    {{-44.5f, 26.0f}, {-44.5f, 21.0f}, LOW_SENSORS},  {{-44.5f, 21.0f}, {-49.5f, 21.0f}, LOW_SENSORS},
    {{-49.5f, 21.0f}, {-49.5f, 26.0f}, LOW_SENSORS},  {{-49.5f, 26.0f}, {-44.5f, 26.0f}, LOW_SENSORS},
    {{-21.0f, 49.5f}, {-21.0f, 44.5f}, LOW_SENSORS},  {{-21.0f, 44.5f}, {-26.0f, 44.5f}, LOW_SENSORS},
    {{-26.0f, 44.5f}, {-26.0f, 49.5f}, LOW_SENSORS},  {{-26.0f, 49.5f}, {-21.0f, 49.5f}, LOW_SENSORS},
    {{44.5f, -26.0f}, {44.5f, -21.0f}, LOW_SENSORS},  {{44.5f, -21.0f}, {49.5f, -21.0f}, LOW_SENSORS},
    {{49.5f, -21.0f}, {49.5f, -26.0f}, LOW_SENSORS},  {{49.5f, -26.0f}, {44.5f, -26.0f}, LOW_SENSORS},
    {{21.0f, -49.5f}, {21.0f, -44.5f}, LOW_SENSORS},  {{21.0f, -44.5f}, {26.0f, -44.5f}, LOW_SENSORS},
    {{26.0f, -44.5f}, {26.0f, -49.5f}, LOW_SENSORS},  {{26.0f, -49.5f}, {21.0f, -49.5f}, LOW_SENSORS},
};

inline const std::vector<FBLIB::MclTracking::CircleObstacle> fieldCircles = {
    {0.0f, 0.0f, 1.594f, HIGH_SENSORS},  // centre base post, above the low part
};

// Goal bases block wall readings for RCL resets: {x, y, radius}
struct BaseCircle {
    float x, y, radius;
};
inline const std::vector<BaseCircle> rclBlockers = {
    {0.0f, 0.0f, 4.0f},   {-47.0f, 23.5f, 4.0f}, {-23.5f, 47.0f, 4.0f},   // neutral bases
    {47.0f, -23.5f, 4.0f}, {23.5f, -47.0f, 4.0f},
    {23.5f, 47.0f, 4.0f},  {47.0f, 23.5f, 4.0f},                          // alliance bases
    {-23.5f, -47.0f, 4.0f}, {-47.0f, -23.5f, 4.0f},
};

}  // namespace robot
