#include "main.h"
#include "FBLib/FB_API.hpp"

#include "FBLib/util/Util.hpp"
#include "pros/llemu.hpp"   // brain screen debug output
#include "pros/motors.hpp"   // motor_brake_mode_e
#include "pros/rtos.hpp"     // pros::delay, pros::Task

using namespace FBLIB;

// 
// Hardware definitions
// 

// Controller
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
inline pros::MotorGroup leftMotors({-11, -12, 13}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({20, 19, -18}, pros::MotorGearset::blue);
inline pros::Motor frontMotor(17, pros::MotorGearset::blue);
inline pros::Motor leverMotor(-16, pros::MotorGearset::red);

// IMU
inline ScaledIMU imu(15, 360.0, 354.25); // Adjust actual_reading based on your IMU's behavior

// Optical
inline pros::Optical frontOptic(3);

// Pneumatics
inline pros::adi::Pneumatics matchLoadGate('D', false, false);
inline pros::adi::Pneumatics lift('C', true, true);
inline pros::adi::Pneumatics leftDescoreArm('A', false, false);
inline pros::adi::Pneumatics trapDoor('B', false, false);
inline pros::adi::Pneumatics intakeLift('G', true, true);

//
// Drivetrain — must be declared before OdomSensors (odometry references it)
//

Drivetrain drivetrain(&leftMotors, &rightMotors,
                      10.4f,   // track width (inches)
                      3.25f,   // wheel diameter (inches)
                      450.0f,  // RPM
                      2.0f);   // horizontal drift factor

//
// Chassis configuration
//

// Sensor collection for odometry
// Fallback chain for each axis:
//   Vertical:  tracking wheels → motor encoders → 0
//   Horizontal: tracking wheels → 0 (tank drives can't strafe)
//   Heading:   IMU → 0
// Set .drivetrain to enable motor encoder fallback when tracking wheels are absent.
OdomSensors odomSensors = {
    .vertWheelCollection  = {},
    .horizWheelCollection = {},
    .imuCollection        = { &imu },
    .drivetrain = &drivetrain   // motor encoder fallback — no tracking wheels
};

// Distance sensors for MCL/RCL tracking (indices 0-3 active, 4-7 unused)
inline pros::Distance front_dist(1);
inline pros::Distance left_dist(2);
inline pros::Distance back_dist(14);
inline pros::Distance right_dist(8);
std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &front_dist, &left_dist, &back_dist, &right_dist,
    nullptr, nullptr, nullptr, nullptr
};

// Sensor mount offsets (robot-frame: x=forward, y=left, theta=pointing angle)
// These MUST be measured for your specific robot.
std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts = {{
    {  6.0f,  0.0f,  0.0f       },  // [0] FRONT  — points forward
    {  0.0f,  3.5f,  HALF_PI    },  // [1] LEFT   — points left
    { -5.0f,  0.0f,  PI         },  // [2] BACK   — points backward
    {  0.0f, -3.5f,  3.0f*HALF_PI }, // [3] RIGHT  — points right
    // [4]–[7] unused (all zeros — library ignores null distance sensor slots)
}};

// Full chassis configuration
ChassisConfig chassisConfig = {
    // PID gains — tuned for 450 RPM, 3.25" wheels, 10.4" track
    // Angular errors are in radians (0 to ±π), so P needs to be higher
    // than lateral P (which sees inch-scale errors).
    .lateralGains     = { 5.0f, 0.0f, 5.0f },
    .angularGains     = { 50.0f, 0.0f, 6.0f },
    .lateralWindupRange = 0.0f,
    .angularWindupRange = 0.0f,

    // Joystick curves
    .throttleCurve    = { .deadband = 10.0f, .minOutput = 15.0f, .curve = 1.2f },
    .steerCurve       = { .deadband = 10.0f, .minOutput = 10.0f, .curve = 1.3f },

    // Distance sensors and mount offsets (user-declared above, stored here)
    .distanceSensors  = distSensors,
    .sensorMounts     = sensorMounts,

    // Tracking systems (enable in competition, off for testing)
    .useMclTracking   = false,
    .useRclTracking   = false,
};

//
// The Chassis object
//

Chassis chassis(drivetrain, odomSensors, chassisConfig);

// 
// PROS Lifecycle
// 

/// Runs once on startup. Calibrate sensors, build Auton Selector UI.
void initialize() {
	pros::lcd::initialize();

    // Set brake mode on all drive motors (hold position when stopped)
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // Calibrate IMU and zero tracking wheels (~2 seconds)
    chassis.calibrate();

	pros::lcd::print(0, "FBLib Initialized!");

    // Set starting pose (match loader position depends on alliance)
    // For skills: center of field, facing +X
    // chassis.setPose({ 0.0f, 0.0f, 0.0f });

    // — Register autonomous routines with the selector —
    // No manual waypoints: paths are auto-generated via dry-run.
    // Skills goes first so it's the default selection.
    //selector.registerAuton("Skills",          autonSkills);
    //selector.registerAuton("Red Offensive",   autonRedOffensive);
    //selector.registerAuton("Red Defensive",   autonRedDefensive);
    //selector.registerAuton("Blue Offensive",  autonBlueOffensive);
    //selector.registerAuton("Blue Defensive",  autonBlueDefensive);

    // — Wire up Auton Selector UI —
    //selector.init();

    // — Set up Path Preview on the brain screen —
    //pathPreview.init();

    // Enable automatic dry-run path generation.
    // When the user changes the selected auton, the library runs it in dry-run
    // mode (motors disabled, pose simulated) and draws the trajectory.
    // The auton's own setPose() call (if any) sets the starting position;
    // if the auton doesn't call setPose, the path starts from (0,0,0).
    //selector.enableDryRun(chassis, pathPreview);
}

/// Runs continuously while robot is disabled (before and between matches).
/// Update auton selector UI, let user pick routine + alliance.
/// Path preview is auto-generated via dry-run when the user changes the selection.
void disabled() {
    while (true) {
        // — Optional: Animate the recorded dry-run path —
        // pathPreview.animate(10);

        pros::delay(20);
    }
}

/// Runs once when competition starts (before autonomous).
void competition_initialize() {
    // Recalibrate IMU in case robot was moved
    chassis.calibrate();

    // Start tracking systems for competition
    // chassis.mcl().startTracking();
    // chassis.rcl().startTracking();
}

/// 15-second autonomous period.
void autonomous() {
    // Run whatever the user selected on the brain screen
    // selector.runSelected();

    // If the selected routine is empty (no autons registered), do nothing
    // and let the 15 seconds expire.
	pros::lcd::print(1, "Running autonomous!");
	chassis.moveDistance(48.0f, 2000, { .forwards = true, .maxSpeed = 100.0f });
}

/// Driver control period (1:45).
void opcontrol() {
    // Stop any tracking that was running
    chassis.stopTracking();

    // Driver control loop
    while (true) {
        // Read joystick values (always read — arcade() yields to async motions internally)
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // — Choose one drive style —

        // Option A: Tank drive (left stick = left wheels, right stick = right wheels)
        // int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // chassis.tank(leftY, rightY);

        // Option B: Arcade drive (left stick = throttle, right stick = steer)
        chassis.arcade(leftY, rightX);

        // Option C: Curvature drive (smoother turning while driving)
        // chassis.curvature(leftY, rightX);

        // — Macro buttons —
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.turnToHeading(stdRadToVexDeg(chassis.getPose().theta)+90.0f, 2000, { .maxSpeed = 80.0f }, true);  // async
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            chassis.moveDistance(24.0f, 2000, {}, true);
        }

        // Cancel any async movement
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.cancelMotion();
        }

        // Toggle brake mode
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            static bool hold = true;
            hold = !hold;
            chassis.setBrakeMode(hold
                ? pros::E_MOTOR_BRAKE_HOLD
                : pros::E_MOTOR_BRAKE_COAST);
            master.rumble(".");
        }

        pros::delay(10);
		pros::lcd::print(1, "Running opcontrol!");
        pros::lcd::print(2, "X:%f Y:%f T:%f", chassis.getPose().x, chassis.getPose().y, stdRadToVexDeg(chassis.getPose().theta));
    }
}
