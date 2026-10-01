#pragma once

// ============================================================================
// Driver control
// ============================================================================
//
// Sticks: tank drive (left Y = left side, right Y = right side).
//   L1      release the cup (outtake 300 ms), then swing the effector up
//   R1      hold: effector square, cup held, lift up once the effector is
//           clear; above 27.5" the effector tips to the toggle angle
//   R2      hold: lift down, cup held; below 27.5" the effector squares again
//           (on release of R1/R2 the lift holds that height)
//   L2      hold: pick up — lift and effector down, both intakes in
//   B       hold: both intakes out
//   Down    hold: toggle — effector to the toggle angle, effector intake out
//   Y       hold: effector to the pin angle
// ============================================================================

namespace robot {

/// Call once at the start of opcontrol(). Stops whatever autonomous left
/// running: a motion would lock out the sticks, and a scoring macro would
/// move the lift by itself.
void startDriverControl();

/// One step of driver control. Call every 20 ms from opcontrol().
void driverControlStep();

}  // namespace robot
