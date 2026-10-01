#include "robot/mechanisms.hpp"

#include <algorithm>
#include <cstdint>
#include <mutex>

#include "pros/rtos.hpp"
#include "robot/config.hpp"

namespace robot {

namespace {

constexpr uint32_t LOOP_MS = 20;

// Effector homing: wait, then drive into the hard stop and call that 0
constexpr uint32_t HOME_SETTLE_MS = 200;
constexpr uint32_t HOME_DRIVE_MS = 1000;

// Holding the lift never pulls it down harder than this error asks for, so
// lowering is gentle (motor degrees)
constexpr float LIFT_MAX_DOWN_ERROR = -60.0f;

// scoreCup: the lift counts as down within this of the drop height (motor
// degrees). If the cup or base stops it short, release anyway after the timeout.
constexpr float SCORE_DROP_MARGIN = 100.0f;
constexpr uint32_t SCORE_DROP_TIMEOUT_MS = 1500;
constexpr uint32_t SCORE_RELEASE_MS = 100;

constexpr int EFFECTOR_HOLD_POWER = 50;
constexpr int EFFECTOR_OUTTAKE_RPM = 140;

// Converted from the LemLib PIDs (kP 1.2, kD 0.5 per 20 ms tick): FBLib's
// derivative is per second. Errors are in motor degrees.
FBLIB::PID effectorPID(1.2f, 0.0f, 0.01f);
FBLIB::PID liftPID(1.2f, 0.0f, 0.01f);

enum class Homing { OFF, SETTLE, DRIVE };
enum class IntakeEnd { STOP, RAISE_EFFECTOR };
enum class Score { OFF, DROPPING, RELEASING };

// Shared with the mechanism task. Callers only touch it briefly under the
// mutex: a competition task stopped mid-call must not leave it locked.
pros::Mutex stateMutex;
struct State {
    float effectorTarget = 0.0f;  // effector degrees
    Homing homing = Homing::OFF;
    uint32_t homingStart = 0;

    float liftTarget = 0.0f;      // inches
    bool liftHold = false;
    bool liftHoldEngaged = false;

    bool intakeMacro = false;     // timed effector-intake outtake
    uint32_t intakeMacroEnd = 0;
    IntakeEnd intakeMacroThen = IntakeEnd::STOP;

    Score score = Score::OFF;     // scoreCup
    float scoreDrop = 0.0f, scoreRaise = 0.0f;
    uint32_t scoreStepStart = 0;
} state;

pros::Task* mechanismTask = nullptr;

bool previewing() { return chassis.isDryRun(); }

int clampPower(float value) {
    return static_cast<int>(std::clamp(value, -127.0f, 127.0f));
}

// ---- Mechanism task (caller holds stateMutex) ----

void stepEffector(uint32_t now, float dt) {
    switch (state.homing) {
    case Homing::SETTLE:
        effectorRotateMotor.move(0);
        if (now - state.homingStart >= HOME_SETTLE_MS) {
            state.homing = Homing::DRIVE;
            state.homingStart = now;
        }
        return;
    case Homing::DRIVE:
        effectorRotateMotor.move(-127);
        if (now - state.homingStart >= HOME_DRIVE_MS) {
            effectorRotateMotor.set_zero_position(0.0);
            effectorRotateMotor.move(0);
            effectorPID.reset();
            state.homing = Homing::OFF;
        }
        return;
    case Homing::OFF:
        break;
    }
    const float error = state.effectorTarget * EFFECTOR_GEAR_RATIO -
                        static_cast<float>(effectorRotateMotor.get_position());
    effectorRotateMotor.move(clampPower(effectorPID.update(error, dt)));
}

void stepLift(float dt) {
    if (!state.liftHold) {
        state.liftHoldEngaged = false;
        return;
    }
    if (!state.liftHoldEngaged) {
        liftPID.reset();
        state.liftHoldEngaged = true;
    }
    float error = state.liftTarget * LIFT_GEAR_RATIO - static_cast<float>(liftMotors.get_position());
    error = std::max(error, LIFT_MAX_DOWN_ERROR);
    liftMotors.move(clampPower(liftPID.update(error, dt)));
}

void stepMacros(uint32_t now) {
    if (state.intakeMacro && static_cast<int32_t>(now - state.intakeMacroEnd) >= 0) {
        state.intakeMacro = false;
        if (state.intakeMacroThen == IntakeEnd::RAISE_EFFECTOR) {
            state.effectorTarget = EFFECTOR_ANGLES[HIGH_ANGLE];  // intake keeps reversing
        } else {
            effectorIntakeMotor.move(0);
        }
    }

    switch (state.score) {
    case Score::DROPPING: {
        const bool down = liftMotors.get_position() <= state.scoreDrop * LIFT_GEAR_RATIO + SCORE_DROP_MARGIN;
        if (down || now - state.scoreStepStart >= SCORE_DROP_TIMEOUT_MS) {
            effectorIntakeMotor.move_velocity(-EFFECTOR_OUTTAKE_RPM);  // release the cup
            state.score = Score::RELEASING;
            state.scoreStepStart = now;
        }
        break;
    }
    case Score::RELEASING:
        if (now - state.scoreStepStart >= SCORE_RELEASE_MS) {
            state.liftTarget = state.scoreRaise;
            state.score = Score::OFF;
        }
        break;
    case Score::OFF:
        break;
    }
}

void mechanismLoop() {
    uint32_t last = pros::millis();
    while (true) {
        const uint32_t now = pros::millis();
        const float dt = static_cast<float>(now - last) / 1000.0f;
        last = now;
        {
            std::lock_guard<pros::Mutex> lock(stateMutex);
            stepEffector(now, dt);
            stepLift(dt);
            stepMacros(now);
        }
        pros::delay(LOOP_MS);
    }
}

void startIntakeMacro(uint32_t ms, IntakeEnd then) {
    if (previewing()) return;
    effectorIntakeMotor.move_velocity(-EFFECTOR_OUTTAKE_RPM);
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.intakeMacro = true;
    state.intakeMacroEnd = pros::millis() + ms;
    state.intakeMacroThen = then;
}

}  // namespace

void initMechanisms() {
    if (mechanismTask != nullptr) return;
    effectorRotateMotor.set_encoder_units(pros::MotorEncoderUnits::degrees);
    // The _all versions: the plain MotorGroup setters only change the first motor
    liftMotors.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
    liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    liftMotors.set_zero_position_all(0.0);
    mechanismTask = new pros::Task([] { mechanismLoop(); }, TASK_PRIORITY_DEFAULT,
                                   TASK_STACK_DEPTH_DEFAULT, "mechanisms");
}

// ---- Intakes ----

void startFrontIntake(int power) {
    if (!previewing()) frontIntakeMotor.move(power);
}
void reverseFrontIntake(int power) {
    if (!previewing()) frontIntakeMotor.move(-power);
}
void stopFrontIntake() {
    if (!previewing()) frontIntakeMotor.move(0);
}
void startEffectorIntake(int rpm) {
    if (!previewing()) effectorIntakeMotor.move_velocity(rpm);
}
void reverseEffectorIntake(int rpm) {
    if (!previewing()) effectorIntakeMotor.move_velocity(-rpm);
}
void holdEffectorIntake() {
    if (!previewing()) effectorIntakeMotor.move(EFFECTOR_HOLD_POWER);
}
void stopEffectorIntake() {
    if (!previewing()) effectorIntakeMotor.move(0);
}

// ---- End effector ----

void setEffector(EffectorAngle angle) {
    setEffector(EFFECTOR_ANGLES[angle]);
}
void setEffector(float degrees) {
    if (previewing()) return;
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.effectorTarget = degrees;
}
void resetEffector() {
    setEffector(0.0f);
}
void homeEffector() {
    if (previewing()) return;
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.effectorTarget = 0.0f;
    state.homing = Homing::SETTLE;
    state.homingStart = pros::millis();
}
float effectorDegrees() {
    return static_cast<float>(effectorRotateMotor.get_position()) / EFFECTOR_GEAR_RATIO;
}

// ---- Lift ----

void setLift(LiftHeight height) {
    setLift(LIFT_HEIGHTS[height]);
}
void setLift(float inches) {
    if (previewing()) return;
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.liftTarget = inches;
}
void resetLift() {
    setLift(0.0f);
}
void setLiftHold(bool hold) {
    if (previewing()) return;
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.liftHold = hold;
}
void driveLift(int power) {
    if (previewing()) return;
    {
        std::lock_guard<pros::Mutex> lock(stateMutex);
        state.liftHold = false;
    }
    liftMotors.move(power);
}
float liftInches() {
    return static_cast<float>(liftMotors.get_position()) / LIFT_GEAR_RATIO;
}

// ---- Macros ----

void scoreCup(float dropHeight, float raiseHeight) {
    if (previewing()) return;
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.liftTarget = dropHeight;
    state.liftHold = true;
    state.score = Score::DROPPING;
    state.scoreDrop = dropHeight;
    state.scoreRaise = raiseHeight;
    state.scoreStepStart = pros::millis();
}
void scorePin() {
    startIntakeMacro(400, IntakeEnd::STOP);
}
void effectorToggle() {
    startIntakeMacro(800, IntakeEnd::STOP);
}
void releaseAndRaiseEffector() {
    startIntakeMacro(300, IntakeEnd::RAISE_EFFECTOR);
}
bool effectorIntakeBusy() {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    return state.intakeMacro;
}
void cancelMacros() {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    state.intakeMacro = false;
    state.score = Score::OFF;
}

}  // namespace robot
