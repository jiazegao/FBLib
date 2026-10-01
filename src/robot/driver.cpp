#include "robot/driver.hpp"

#include "robot/config.hpp"
#include "robot/mechanisms.hpp"

namespace robot {

namespace {

// The lift only rises once the effector has swung this far (clear of the
// lift), and above TOGGLE_LIFT_HEIGHT the effector tips to the toggle angle
constexpr float EFFECTOR_CLEAR_ANGLE = 80.0f;  // degrees (320 motor degrees)
constexpr float TOGGLE_LIFT_HEIGHT = 27.5f;    // inches (3300 motor degrees)

constexpr int LIFT_UP_POWER = 127;
constexpr int LIFT_DOWN_POWER = 60;

bool held(pros::controller_digital_e_t button) {
    return controller.get_digital(button) != 0;
}

}  // namespace

void startDriverControl() {
    chassis.cancelMotion();
    cancelMacros();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void driverControlStep() {
    // Ignored while a motion is running (e.g. one left over from autonomous)
    chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                 controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

    // Read the edges every step: each press or release is reported only once
    const bool l1Pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) != 0;
    const bool r1Released = controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1) != 0;
    const bool r2Released = controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2) != 0;
    const bool r1 = held(pros::E_CONTROLLER_DIGITAL_R1);
    const bool r2 = held(pros::E_CONTROLLER_DIGITAL_R2);

    if (l1Pressed) releaseAndRaiseEffector();

    // ---- Lift ----
    if (r1) {
        setEffector(RIGHT_ANGLE);
        holdEffectorIntake();
        driveLift(effectorDegrees() > EFFECTOR_CLEAR_ANGLE ? LIFT_UP_POWER : 0);
        if (liftInches() > TOGGLE_LIFT_HEIGHT) setEffector(TOGGLE_ANGLE);
    } else if (r2) {
        holdEffectorIntake();
        driveLift(-LIFT_DOWN_POWER);
        if (liftInches() < TOGGLE_LIFT_HEIGHT) setEffector(RIGHT_ANGLE);
    } else {
        // Hold the height R1/R2 let go at. Set the target before the hold
        // engages, or it would pull toward the old target for a moment.
        if (r1Released || r2Released) setLift(liftInches());
        setLiftHold(true);
    }

    // ---- Intakes and effector ----
    if (held(pros::E_CONTROLLER_DIGITAL_L2)) {
        setLiftHold(true);
        resetLift();
        resetEffector();
        startFrontIntake();
        startEffectorIntake();
    } else if (held(pros::E_CONTROLLER_DIGITAL_B)) {
        reverseFrontIntake();
        reverseEffectorIntake();
    } else if (held(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        setEffector(TOGGLE_ANGLE);
        stopFrontIntake();
        reverseEffectorIntake();
    } else {
        stopFrontIntake();
        const bool busy = effectorIntakeBusy();
        // R1 and R2 keep holding the cup; an L1 release runs to its end
        if (!busy && !r1 && !r2) stopEffectorIntake();
        if (held(pros::E_CONTROLLER_DIGITAL_Y) && !busy && !r1 && !r2 &&
            !held(pros::E_CONTROLLER_DIGITAL_L1)) {
            setEffector(PIN_ANGLE);
        }
    }
}

}  // namespace robot
