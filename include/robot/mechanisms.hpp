#pragma once

// ============================================================================
// Mechanisms — intakes, end effector and lift
// ============================================================================
//
// One background task (started by initMechanisms()) holds the effector and
// lift on their targets and runs the timed macros, so the functions below
// return immediately and are safe to call from any task.
//
// During a selector preview (chassis.isDryRun()) they do nothing: the preview
// runs the routine's code, but should only move the simulated drivetrain.
// ============================================================================

namespace robot {

// End effector presets (degrees at the effector, 0 = homed against its stop)
enum EffectorAngle { IDLE_ANGLE, LOW_ANGLE, RIGHT_ANGLE, TOGGLE_ANGLE, HIGH_ANGLE, PIN_ANGLE };
constexpr float EFFECTOR_ANGLES[6] = {0.0f, 60.0f, 90.0f, 105.0f, 165.0f, 86.0f};

// Lift presets (inches above the bottom)
enum LiftHeight { IDLE_HEIGHT, FIRST_STACK, SECOND_STACK, THIRD_STACK, FOURTH_STACK, FIFTH_STACK };
constexpr float LIFT_HEIGHTS[6] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f};

constexpr float EFFECTOR_GEAR_RATIO = 4.0f;  // motor degrees per effector degree
constexpr float LIFT_GEAR_RATIO = 120.0f;    // motor degrees per inch of lift

/// Set up the mechanism motors and start the mechanism task. Call once from
/// initialize(), with the lift all the way down (that height becomes 0).
void initMechanisms();

// ---- Intakes ----
void startFrontIntake(int power = 127);
void reverseFrontIntake(int power = 127);
void stopFrontIntake();
void startEffectorIntake(int rpm = 200);
void reverseEffectorIntake(int rpm = 140);
void holdEffectorIntake();   // gentle pull that keeps a cup in while lifting
void stopEffectorIntake();

// ---- End effector ----
void setEffector(EffectorAngle angle);
void setEffector(float degrees);
void resetEffector();        // back to 0
/// Drive the effector into its hard stop and call that 0 (~1.2 s). A target
/// set meanwhile is applied once homing is done.
void homeEffector();
float effectorDegrees();     // measured angle

// ---- Lift ----
void setLift(LiftHeight height);
void setLift(float inches);
void resetLift();            // back to 0
/// Hold the lift at its target (on), or leave it to driveLift() (off)
void setLiftHold(bool hold);
void driveLift(int power);   // manual power, -127..127 (with the hold off)
float liftInches();          // measured height

// ---- Macros (run in the mechanism task) ----
/// Lower the lift to dropHeight, release the cup (the effector intake keeps
/// reversing), then lift to raiseHeight. Raise the lift and square the
/// effector first.
void scoreCup(float dropHeight, float raiseHeight);
/// Outtake the effector intake for 400 ms (effector square to the pin first)
void scorePin();
/// Outtake the effector intake for 800 ms
void effectorToggle();
/// Outtake the effector intake for 300 ms, then swing the effector up
void releaseAndRaiseEffector();
/// True while a macro is running the effector intake
bool effectorIntakeBusy();
/// Drop any running macros (e.g. leftovers from autonomous)
void cancelMacros();

}  // namespace robot
