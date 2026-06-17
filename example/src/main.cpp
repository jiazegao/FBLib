#include "main.h"
#include "FBLib/FB_API.hpp"

#include "pros/llemu.hpp"   // brain screen debug output
#include "pros/motors.hpp"   // motor_brake_mode_e
#include "pros/rtos.hpp"     // pros::delay, pros::Task

using namespace FBLIB;

// ============================================================================
// Hardware definitions
// ============================================================================

// Drive motors — 2 per side, 600 RPM cartridges
pros::MotorGroup leftMotors({1, 2});
pros::MotorGroup rightMotors({3, 4});

// IMU
pros::Imu imu(5);

// ADI encoders for tracking wheels
// Ports A (top wire) and B (bottom wire) — vertical wheel
// Ports C (top) and D (bottom) — horizontal wheel
pros::adi::Encoder vertEnc('A', 'B');
pros::adi::Encoder horizEnc('C', 'D');

// Tracking wheels — 2.75" diameter, direct drive (1:1)
TrackingWheel vertWheel(vertEnc, 2.75f, 0.0f);   // centered forward/back
TrackingWheel horizWheel(horizEnc, 2.75f, -3.5f); // 3.5" behind tracking center

// Distance sensors (port, no special config needed)
pros::Distance distFront(6);
pros::Distance distLeft(7);
pros::Distance distBack(8);
pros::Distance distRight(9);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ============================================================================
// Chassis configuration
// ============================================================================

// Sensor collection for odometry
// Fallback chain for each axis:
//   Vertical:  tracking wheels → motor encoders → 0
//   Horizontal: tracking wheels → 0 (tank drives can't strafe)
//   Heading:   IMU → 0
// Set .drivetrain to enable motor encoder fallback when tracking wheels are absent.
OdomSensors odomSensors = {
    .vertWheelCollection  = { &vertWheel },
    .horizWheelCollection = { &horizWheel },
    .imuCollection        = { &imu }
    // .drivetrain = &drivetrain   // uncomment if you have no tracking wheels
};

// Distance sensors for MCL/RCL tracking (indices 0-3 active, 4-7 unused)
std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distSensors = {
    &distFront, &distLeft, &distBack, &distRight,
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
    // PID gains — tuned for 600 RPM, 3.25" wheels, 10.4" track
    .lateralGains     = { 7.0f, 0.0f, 30.0f },
    .angularGains     = { 4.0f, 0.0f, 37.7f },
    .lateralWindupRange = 20.0f,
    .angularWindupRange = 5.0f,

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

// ============================================================================
// The Chassis object — everything flows through this
// ============================================================================

Drivetrain drivetrain(&leftMotors, &rightMotors,
                      10.4f,   // track width (inches)
                      3.25f,   // wheel diameter (inches)
                      600.0f,  // RPM
                      1.0f);   // horizontal drift factor

Chassis chassis(drivetrain, odomSensors, chassisConfig);

// ============================================================================
// Auton Selector + Path Preview
// ============================================================================

AutonSelector selector;
PathPreview pathPreview;

// ============================================================================
// Autonomous routines
// ============================================================================

// — Skills autonomous — scores on both sides of the field —
void autonSkills() {
    // Start at match loader, heading toward field center (0°)
    // Drive forward to the center stake
    chassis.moveDistance(48.0f, 2000, { .forwards = true, .maxSpeed = 100.0f });

    // Turn to face the right-side mobile goal
    chassis.turnToPoint(60.0f, 12.0f, 1000);

    // Boomerang approach to the goal (curved path that ends facing the goal)
    chassis.moveBoomerang(60.0f, 12.0f, 90.0f, 3000, {
        .maxSpeed = 80.0f,
        .targetTolerance = 2.0f,
        .headingTolerance = 5.0f,
        .lead = 0.6f,
        .leadDecay = 0.4f
    });

    // Drive back to center
    chassis.moveToPoint(0.0f, 0.0f, 0, { .maxSpeed = 100.0f });

    // Turn to face left side
    chassis.turnToHeading(270.0f, 1500, { .maxSpeed = 90.0f });

    // Arc around to the left-side goal
    chassis.moveArc(-60.0f, 12.0f, -24.0f, 0, {
        .maxSpeed = 70.0f,
        .targetTolerance = 2.0f
    });
}

// — Red offensive: rush right side mobile goal —
void autonRedOffensive() {
    // Start facing field center from red match loader
    // Preload: grab mobile goal directly ahead

    // Drive forward, grab goal
    chassis.moveDistance(36.0f, 2000, { .maxSpeed = 127.0f });

    // Pivot turn (left side stationary) to face the corner
    chassis.swingToHeading(135.0f, SwingSide::Left, 1000, {
        .direction = TurnDirection::CW,
        .maxSpeed = 90
    });

    // RAMSETE path following: smooth curve through scoring positions
    std::vector<Pose> scoringPath = {
        {  0.0f,   0.0f, 0.0f },
        { 12.0f, -12.0f, -QUARTER_PI },
        { 24.0f, -24.0f, -HALF_PI },
        { 12.0f, -36.0f, -PI },
    };
    chassis.moveRAMSETE(scoringPath, 5000, {
        .maxSpeed = 100.0f,
        .targetTolerance = 1.5f,
        .headingTolerance = 6.0f,
        .lookaheadDist = 8.0f
    });

    // Face the corner goal
    chassis.turnToPoint(-60.0f, -60.0f, 1000);

    // Final approach
    chassis.moveToPoint(-60.0f, -60.0f, 0, { .maxSpeed = 60.0f, .targetTolerance = 1.0f });
}

// — Red defensive: block blue from center —
void autonRedDefensive() {
    // Rush to center and hold position

    // Fast arc to center-right defensive position
    chassis.moveArc(24.0f, 12.0f, 36.0f, 0, {
        .maxSpeed = 127.0f,
        .targetTolerance = 3.0f
    });

    // Face the blue alliance side
    chassis.turnToHeading(90.0f, 1000, {
        .direction = TurnDirection::SHORTEST,
        .maxSpeed = 100.0f
    });
}

// — Blue offensive: mirror of red offensive —
void autonBlueOffensive() {
    // Everything mirrored across the Y-axis (X → -X)
    chassis.moveDistance(36.0f, 2000, { .maxSpeed = 127.0f });

    chassis.swingToHeading(225.0f, SwingSide::Right, 1000, {
        .direction = TurnDirection::CCW,
        .maxSpeed = 90.0f
    });

    std::vector<Pose> scoringPath = {
        {  0.0f,   0.0f,  0.0f },
        { -12.0f, -12.0f, -3.0f * QUARTER_PI },
        { -24.0f, -24.0f, -HALF_PI },
        { -12.0f, -36.0f, -PI },
    };
    chassis.moveRAMSETE(scoringPath, 5000, {
        .maxSpeed = 100.0f,
        .targetTolerance = 1.5f,
        .headingTolerance = 6.0f,
        .lookaheadDist = 8.0f,
    });

    chassis.turnToPoint(60.0f, -60.0f, 1000);
    chassis.moveToPoint(60.0f, -60.0f, 0, { .maxSpeed = 60.0f, .targetTolerance = 1.0f });
}

// — Blue defensive —
void autonBlueDefensive() {
    chassis.moveArc(-24.0f, 12.0f, -36.0f, 0, {
        .maxSpeed = 127.0f,
        .targetTolerance = 3.0f
    });
    chassis.turnToHeading(270.0f, 1000, { .maxSpeed = 100.0f });
}

// ============================================================================
// PROS Lifecycle
// ============================================================================

/// Runs once on startup. Calibrate sensors, build Auton Selector UI.
void initialize() {
    // Set brake mode on all drive motors (hold position when stopped)
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    // Calibrate IMU and zero tracking wheels (~2 seconds)
    chassis.calibrate();

    // Set starting pose (match loader position depends on alliance)
    // For skills: center of field, facing +X
    chassis.setPose({ 0.0f, 0.0f, 0.0f });

    // — Register autonomous routines with the selector —
    // No manual waypoints: paths are auto-generated via dry-run.
    // Skills goes first so it's the default selection.
    selector.registerAuton("Skills",          autonSkills);
    selector.registerAuton("Red Offensive",   autonRedOffensive);
    selector.registerAuton("Red Defensive",   autonRedDefensive);
    selector.registerAuton("Blue Offensive",  autonBlueOffensive);
    selector.registerAuton("Blue Defensive",  autonBlueDefensive);

    // — Wire up Auton Selector UI —
    selector.init();

    // — Set up Path Preview on the brain screen —
    pathPreview.init();

    // Enable automatic dry-run path generation.
    // When the user changes the selected auton, the library runs it in dry-run
    // mode (motors disabled, pose simulated) and draws the trajectory.
    // The auton's own setPose() call (if any) sets the starting position;
    // if the auton doesn't call setPose, the path starts from (0,0,0).
    selector.enableDryRun(chassis, pathPreview);

    // Print confirmation to brain screen
    pros::lcd::initialize();
    pros::lcd::print(0, "FBLib initialized");
    pros::lcd::print(1, "Autons: %d  Alliance: NONE", selector.count());
}

/// Runs continuously while robot is disabled (before and between matches).
/// Update auton selector UI, let user pick routine + alliance.
/// Path preview is auto-generated via dry-run when the user changes the selection.
void disabled() {
    while (true) {
        selector.update();

        // — Optional: Animate the recorded dry-run path —
        // pathPreview.animate(10);

        pros::delay(20);
    }
}

/// Runs once when competition starts (before autonomous).
void competition_initialize() {
    // Recalibrate IMU in case robot was moved
    chassis.calibrate();

    // Set starting pose based on selected alliance
    Alliance alliance = selector.getAlliance();
    if (selector.isSkills()) {
        chassis.setPose({ -60.0f, 0.0f, 0.0f });  // skills: match loader, facing +X
    } else if (alliance == Alliance::RED) {
        chassis.setPose({ -60.0f, 0.0f, 0.0f });  // red match loader
    } else if (alliance == Alliance::BLUE) {
        chassis.setPose({ 60.0f, 0.0f, PI });     // blue match loader, facing -X
    }

    // Start tracking systems for competition
    // chassis.mcl().startTracking();
    // chassis.rcl().startTracking();

    pros::lcd::print(2, "Competition start — %s",
        selector.isSkills() ? "Skills" : "Match");
}

/// 15-second autonomous period.
void autonomous() {
    // Run whatever the user selected on the brain screen
    selector.runSelected();

    // If the selected routine is empty (no autons registered), do nothing
    // and let the 15 seconds expire.
}

/// Driver control period (1:45).
void opcontrol() {
    // Stop any tracking that was running
    chassis.stopTracking();

    // Driver control loop
    while (true) {
        // Read joystick values
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
        // Turn to 90° heading (facing +X, toward opposing alliance wall)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            chassis.turnToHeading(90.0f, 0, { .maxSpeed = 80.0f }, true);  // async
        }

        // Turn to 270° heading (facing -X, toward home alliance wall)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            chassis.turnToHeading(270.0f, 0, { .maxSpeed = 80.0f }, true);
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
    }
}
