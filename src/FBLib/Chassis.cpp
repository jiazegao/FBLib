#include "FBLib/Chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "FBLib/Util/FastTrig.hpp"
#include "pros/motors.hpp"

namespace FBLIB {

// ============================================================================
// Drivetrain
// ============================================================================

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                       float trackWidth, float wheelDiameter, float rpm,
                       float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {
    // Odometry's motor-encoder fallback assumes positions are in DEGREES.
    // Enforce it — a user who set rotations/counts elsewhere would silently
    // corrupt distance measurements by a factor of 360.
    if (leftMotors != nullptr) {
        leftMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
    if (rightMotors != nullptr) {
        rightMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

void Drivetrain::setLeftVoltage(float voltage) {
    if (leftMotors != nullptr) {
        leftMotors->move_voltage(static_cast<int32_t>(voltage * 94.488f));  // mV * 127 → 12000mV
    }
}

void Drivetrain::setRightVoltage(float voltage) {
    if (rightMotors != nullptr) {
        rightMotors->move_voltage(static_cast<int32_t>(voltage * 94.488f));
    }
}

void Drivetrain::setVoltage(float left, float right) {
    setLeftVoltage(left);
    setRightVoltage(right);
}

void Drivetrain::setBrakeMode(pros::motor_brake_mode_e mode) {
    if (leftMotors != nullptr) leftMotors->set_brake_mode(mode);
    if (rightMotors != nullptr) rightMotors->set_brake_mode(mode);
}

float Drivetrain::averagePositionDeg() const {
    if (leftMotors == nullptr && rightMotors == nullptr) return 0.0f;

    // A disconnected motor reports PROS_ERR (INT32_MAX ≈ 2.1e9 "degrees").
    // One bad reading would catapult the average — and odometry with it —
    // thousands of miles. Skip error readings; average the healthy motors.
    auto accumulate = [](const std::vector<double>& positions,
                         double& sum, int& count) {
        for (double pos : positions) {
            if (!std::isfinite(pos) || std::fabs(pos) >= 2147483647.0) continue;
            sum += pos;
            count++;
        }
    };

    double sum = 0.0;
    int count = 0;
    if (leftMotors != nullptr)  accumulate(leftMotors->get_position_all(),  sum, count);
    if (rightMotors != nullptr) accumulate(rightMotors->get_position_all(), sum, count);
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
}

float Drivetrain::externalGearRatio() const {
    if (mExtGearRatio > 0.0f) return mExtGearRatio;   // cached

    // Resolve cartridge RPM from the configured gearset. get_gearing() can
    // return invalid before the motor responds (e.g. during boot) — in that
    // case return 1.0 WITHOUT caching so a later call can retry.
    pros::MotorGroup* group = (leftMotors != nullptr) ? leftMotors : rightMotors;
    if (group == nullptr || rpm <= 0.0f) return 1.0f;

    float cartridgeRpm;
    switch (group->get_gearing()) {
        case pros::MotorGears::red:   cartridgeRpm = 100.0f; break;
        case pros::MotorGears::green: cartridgeRpm = 200.0f; break;
        case pros::MotorGears::blue:  cartridgeRpm = 600.0f; break;
        default:                      return 1.0f;   // unknown — retry later
    }

    mExtGearRatio = rpm / cartridgeRpm;
    return mExtGearRatio;
}

// ============================================================================
// DriveCurve
// ============================================================================

float DriveCurve::apply(float input) const {
    if (std::fabs(input) < deadband) return 0.0f;

    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    float absInput = std::fabs(input);

    // Remap [deadband, 127] → [0, 1]
    float normalized = (absInput - deadband) / (127.0f - deadband);

    // Apply curve: x^curve
    float curved = std::pow(normalized, curve);

    // Remap back: [minOutput, 127]
    return sign * (minOutput + curved * (127.0f - minOutput));
}

// ============================================================================
// Chassis
// ============================================================================

Chassis::Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
                 const ChassisConfig& config)
    : mDrivetrain(drivetrain),
      mConfig(config),
      mOdom(sensors),
      mRcl(mOdom, config.rclConfig),
      mMcl(mOdom, config.distanceSensors, Pose{}, config.mclConfig),
      mLateralPID(config.lateralGains, config.lateralWindupRange, config.lateralFlipReset),
      mAngularPID(config.angularGains, config.angularWindupRange, config.angularFlipReset),
      mThrottleCurve(config.throttleCurve),
      mSteerCurve(config.steerCurve)
{
    // Apply user-configured sensor mount offsets (robot-specific)
    mMcl.setSensorMounts(config.sensorMounts);

    // Auto-configure RCL sensors from config arrays.
    // Null distance sensor entries are automatically disabled;
    // sensorCount is set to the count of non-null entries.
    mRcl.configureSensors(config.distanceSensors, config.sensorMounts);

    // Start the persistent motion task (created once, reused for all async motions)
    ensureMotionTask();

    // Start the background odometry task — the SOLE caller of mOdom.update().
    // Centralizing updates in one task fixes two concurrency bugs:
    //   1. Data race: motion loop, MCL, and RCL used to all call update()
    //      concurrently, corrupting mPose/mPrev* (unserialized RMW).
    //   2. Missed deltas: every update() call consumes the sensor baseline,
    //      so MCL's per-tick getLastDelta() lost the motion consumed by other
    //      callers' ticks — particles systematically fell behind the robot.
    // It also keeps the pose fresh during driver control, when no motion
    // command is running (previously the pose froze outside of motions).
    mOdomTaskShouldStop = false;
    mOdomTask = new pros::Task([this]() { runOdomTask(); });
}

void Chassis::joinTask(pros::Task*& task, uint32_t timeoutMs) {
    if (task == nullptr) return;
    // Wait for the task to exit on its own (it self-deletes when its function
    // returns). Hard-killing via remove() is a last resort — removing a task
    // that holds the odometry mutex would deadlock every other consumer.
    uint32_t start = pros::millis();
    while (pros::millis() - start < timeoutMs) {
        uint32_t state = task->get_state();
        if (state == pros::E_TASK_STATE_DELETED ||
            state == pros::E_TASK_STATE_INVALID) break;
        pros::delay(5);
    }
    uint32_t state = task->get_state();
    if (state != pros::E_TASK_STATE_DELETED &&
        state != pros::E_TASK_STATE_INVALID) {
        task->remove();
    }
    delete task;
    task = nullptr;
}

Chassis::~Chassis() {
    // Stop the motion task (blocks in notify_take — wake it so it can exit)
    mTaskShouldStop = true;
    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }
    joinTask(mMotionTask, 200);

    // Stop the odometry task
    mOdomTaskShouldStop = true;
    joinTask(mOdomTask, 200);
}

void Chassis::runOdomTask() {
    while (!mOdomTaskShouldStop) {
        mOdom.update();
        pros::delay(10);
    }
}

// ========================================================================
// Calibration & Pose
// ========================================================================

void Chassis::calibrate() {
    mOdom.calibrate();
}

void Chassis::setPose(const Pose& pose) {
    // Dry-run: only re-baseline the SIMULATION. The preview must be
    // side-effect-free — auton functions call setPose() at their start, and
    // mutating real odometry/MCL/RCL here would teleport the robot's actual
    // localization every time a preview is generated.
    if (mDryRun) {
        mDryRunStartPose = pose;
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
        return;
    }
    mOdom.setPose(pose);
    // Propagate to MCL so particles are reinitialized around the new pose.
    // Without this, MCL particles stay at the old pose when the user changes
    // the chassis pose (e.g. during competition_initialize).
    mMcl.setPose(pose);
    mRcl.setRclPose(pose);
}

void Chassis::setPosition(float x, float y) {
    if (mDryRun) {   // side-effect-free in dry-run — adjust the sim pose only
        mDryRunPose.x = x;
        mDryRunPose.y = y;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{x, y, current.theta};
    mOdom.setPose(newPose);
    // Propagate to tracking systems so MCL particles stay centered and RCL
    // estimate stays synchronized.
    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
}

void Chassis::setHeading(float thetaDeg) {
    if (mDryRun) {   // side-effect-free in dry-run — adjust the sim pose only
        mDryRunPose.theta = vexToStdRad(thetaDeg);
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    Pose current = mOdom.getPose();
    Pose newPose{current.x, current.y, vexToStdRad(thetaDeg)};
    mOdom.setPose(newPose);
    // Propagate to tracking systems — same reason as setPosition().
    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
}

Pose Chassis::getPose() const {
    return mOdom.getPose();
}

// ========================================================================
// Driver Control
// ========================================================================

void Chassis::tank(float left, float right) {
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float l = mThrottleCurve.apply(left);
    float r = mThrottleCurve.apply(right);
    mDrivetrain.setVoltage(l, r);
}

void Chassis::arcade(float throttle, float steer) {
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left  = t + s;
    float right = t - s;

    // Normalize to [-127, 127]
    float maxVal = std::max(std::fabs(left), std::fabs(right));
    if (maxVal > 127.0f) {
        left  = left  * 127.0f / maxVal;
        right = right * 127.0f / maxVal;
    }

    mDrivetrain.setVoltage(left, right);
}

void Chassis::curvature(float throttle, float steer) {
    // Curvature drive: inside wheel slows proportionally during turns
    if (mDryRun) return;
    if (!isSettled()) return;   // yield to running OR queued async motion
    float t = mThrottleCurve.apply(throttle);
    float s = mSteerCurve.apply(steer);

    float left, right;
    if (std::fabs(t) < 5.0f) {
        // Stationary turn
        left = s;
        right = -s;
    } else {
        // Moving turn: reduce inside wheel speed
        float turnScale = std::fabs(s) / 127.0f;
        if (s > 0) {
            // Turning right: slow down right side
            left = t;
            right = t * (1.0f - turnScale * 0.7f);
        } else {
            // Turning left: slow down left side
            left = t * (1.0f - turnScale * 0.7f);
            right = t;
        }
    }

    float maxVal = std::max(std::fabs(left), std::fabs(right));
    if (maxVal > 127.0f) {
        left  = left  * 127.0f / maxVal;
        right = right * 127.0f / maxVal;
    }

    mDrivetrain.setVoltage(left, right);
}

// ========================================================================
// Motion error computation
// ========================================================================

float Chassis::computeLateralError(float targetX, float targetY) const {
    Pose current = mOdom.getPose();
    return distanceToPoint(current, targetX, targetY);
}

float Chassis::computeAngularError(float targetHeadingRad) const {
    Pose current = mOdom.getPose();
    return angleDiffRad(current.theta, targetHeadingRad);
}

float Chassis::computeAngularErrorToPoint(float targetX, float targetY) const {
    Pose current = mOdom.getPose();
    float bearingRad = bearingToPoint(current, targetX, targetY);
    return angleDiffRad(current.theta, bearingRad);
}

// ========================================================================
// Motion execution
// ========================================================================

void Chassis::startMotion() {
    mMotionRunning = true;
}

void Chassis::endMotion() {
    mDrivetrain.setVoltage(0, 0);
    mMotionRunning = false;
    mLateralPID.reset();
    mAngularPID.reset();
}

void Chassis::runMotionTask() {
    while (!mTaskShouldStop) {
        // Block until work arrives (FreeRTOS task notification)
        pros::Task::notify_take(true, TIMEOUT_MAX);
        if (mTaskShouldStop) break;

        // Drain all queued motions. This task is the SOLE executor of motions,
        // so nothing else touches mCurrentMotion / the PIDs / the motors.
        while (true) {
            MotionRequest req;
            bool hasWork = false;
            mQueueMutex.lock();
            if (!mMotionQueue.empty()) {
                req = mMotionQueue.front();
                mMotionQueue.pop();
                hasWork = true;
            }
            mQueueMutex.unlock();

            if (!hasWork) break;

            // If cancel() was called, skip execution but still publish the
            // sequence id so any blocking caller waiting on it is released.
            if (!mMotionCancelled) {
                mCurrentMotion = req;
                executeCurrentMotion();
            }
            mCompletedSeq.store(req.seq);
        }
    }
}

void Chassis::executeCurrentMotion() {
    auto& req = mCurrentMotion;

    // Timestamp when execution actually begins (not when enqueued)
    req.startTime = pros::millis();

    // Dry-run: CONTINUE the simulation from the current simulated pose.
    // Do not re-seed from real odometry (the sim pose is authoritative while
    // dry-running — the real robot isn't moving) and do not clear the path:
    // a preview spans ALL motions in the auton, not just the last one.
    if (mDryRun && mDryRunPath.empty()) {
        mDryRunPath.push_back(mDryRunPose);
    }

    // Precompute max linear speed for dry-run integration
    // v_max = rpm / 60 * wheel_diameter * PI  (inches/second)
    float maxSpeedInPerSec = mDrivetrain.rpm / 60.0f *
        mDrivetrain.wheelDiameter * PI;

    // Convert a Distance motion's relative distance into a FIXED field target
    // ONCE, here at motion start.  This target must NOT be recomputed each loop
    // iteration: re-anchoring it to the live pose every tick keeps the error
    // pinned at the requested distance, so the motion never settles (and with
    // no timeout, the blocking call loops forever and freezes the caller).
    if (req.type == MotionType::Distance) {
        Pose start = mDryRun ? mDryRunPose : mOdom.getPose();
        float dir = req.moveDistParams.forwards ? 1.0f : -1.0f;
        req.targetX = start.x + req.targetDistance * dir * std::cos(start.theta);
        req.targetY = start.y + req.targetDistance * dir * std::sin(start.theta);
    }

    // — RAMSETE: precompute path arc-length + a velocity profile ONCE —
    // ramseteCumLen[i] is the arc length from path[0] to path[i] (inches).
    // ramseteProfile gives the desired forward speed (in/s) as a function of
    // distance travelled along the path, so the robot accelerates off the line
    // and decelerates into the endpoint instead of slamming full speed.
    std::vector<float> ramseteCumLen;
    std::vector<ProfilePoint> ramseteProfile;
    if (req.type == MotionType::RAMSETE) {
        const auto& path = req.ramsetePath;
        ramseteCumLen.resize(path.size(), 0.0f);
        for (size_t i = 1; i < path.size(); ++i) {
            ramseteCumLen[i] = ramseteCumLen[i - 1] +
                distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
        }
        float totalLen = ramseteCumLen.empty() ? 0.0f : ramseteCumLen.back();
        // Peak profile speed (in/s), scaled by the caller's maxSpeed fraction.
        float cruiseInPerSec = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
        if (req.ramseteParams.useVelocityProfile && totalLen > 1e-3f &&
            cruiseInPerSec > 1e-3f && req.ramseteParams.maxAccel > 1e-3f) {
            ramseteProfile = generateTrapezoidal(totalLen, cruiseInPerSec,
                                                 req.ramseteParams.maxAccel);
        }
    }

    mMotionCancelled = false;
    startMotion();

    // Dry-run runs FAST-FORWARDED: simulated time advances 10ms per iteration
    // without sleeping, so a 15s auton previews in tens of milliseconds instead
    // of freezing the caller (e.g. the LVGL button callback) for 15 real
    // seconds. The iteration cap guarantees termination even if a simulated
    // motion never settles.
    float simTimeMs = 0.0f;
    int dryRunIters = 0;
    constexpr int DRYRUN_MAX_ITERS = 60000;  // 10 simulated minutes

    while (mMotionRunning && !mMotionCancelled) {
        // Odometry freshness is provided by the background odometry task —
        // do NOT call mOdom.update() here (it would race and consume deltas).
        Pose pose = mDryRun ? mDryRunPose : mOdom.getPose();

        float lateralOut = 0.0f, angularOut = 0.0f;
        float customLeft = 0.0f, customRight = 0.0f;
        bool settled = false;
        bool useCustomOutput = false;  // Swing / RAMSETE set their own motor outputs
        float latLimit = 127.0f;       // max lateral output (±)
        float angLimit = 127.0f;       // max angular output (±)

        switch (req.type) {
        case MotionType::Distance: {
            latLimit = req.moveDistParams.maxSpeed;
            // Target (req.targetX/Y) is the FIXED field point captured once at
            // motion start (see above the loop).  Drive toward it so the error
            // actually shrinks and the motion can settle.
            float distErr = distanceToPoint(pose, req.targetX, req.targetY);
            lateralOut = mLateralPID.update(distErr);

            float targetBearingRad = bearingToPoint(pose, req.targetX, req.targetY);
            if (!req.moveDistParams.forwards) {
                // Backward: the "front" is the robot's back.  Face AWAY from
                // the target so the negated lateral output drives the back
                // toward it.  The heading error stays small because the
                // reverse bearing is near the robot's natural heading.
                targetBearingRad = wrapRad(targetBearingRad + PI);
                lateralOut = -lateralOut;
            }
            float headingErrRad = angleDiffRad(pose.theta, targetBearingRad);
            angularOut = mAngularPID.update(headingErrRad);
            settled = std::fabs(distErr) < req.moveDistParams.targetTolerance;
            break;
        }
        case MotionType::Point: {
            latLimit = req.movePointParams.maxSpeed;
            float distErr = distanceToPoint(pose, req.targetX, req.targetY);
            lateralOut = mLateralPID.update(distErr);

            float targetBearingRad = bearingToPoint(pose, req.targetX, req.targetY);
            if (!req.movePointParams.forwards) {
                // Backward: face AWAY from target + negate lateral, so the
                // robot's back moves toward the target while the robot
                // looks the other way.
                targetBearingRad = wrapRad(targetBearingRad + PI);
                lateralOut = -lateralOut;
            }
            float headingErrRad = angleDiffRad(pose.theta, targetBearingRad);
            angularOut = mAngularPID.update(headingErrRad);
            settled = distErr < req.movePointParams.targetTolerance;
            break;
        }
        case MotionType::Heading: {
            angLimit = req.turnHeadingParams.maxSpeed;
            float headingErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);
            // Resolve to the requested turn direction (CW, CCW, or SHORTEST).
            // Without this, CW/CCW are ignored and the robot always takes the
            // shortest path — a 350° requested CW turn would be 10° CCW.
            headingErrRad = resolveTurnError(headingErrRad, req.turnHeadingParams.direction);
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            settled = std::fabs(headingErrRad) < degToRad(req.turnHeadingParams.targetTolerance);
            break;
        }
        case MotionType::TurnPoint: {
            angLimit = req.turnPointParams.maxSpeed;
            float targetHeadingRad = bearingToPoint(pose, req.targetX, req.targetY);
            float headingErrRad = angleDiffRad(pose.theta, targetHeadingRad);
            headingErrRad = resolveTurnError(headingErrRad, req.turnPointParams.direction);
            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;
            settled = std::fabs(headingErrRad) < degToRad(req.turnPointParams.targetTolerance);
            break;
        }
        case MotionType::Arc: {
            latLimit = req.arcParams.maxSpeed;
            // Arc movement: follow a circular arc to target point
            // Compute chord vector from current pose to target
            float dx = req.targetX - pose.x;
            float dy = req.targetY - pose.y;
            float chordDist = std::sqrt(dx * dx + dy * dy);

            // Chord bearing
            float chordBearingRad = std::atan2(dy, dx);

            // Arc radius determines curvature (positive = turning left, negative = right)
            float radius = std::fabs(req.targetRadius);
            if (radius < 1e-6f) {
                // Degenerate: straight line to target
                lateralOut = mLateralPID.update(chordDist);
                float headingErrRad = angleDiffRad(pose.theta, chordBearingRad);
                angularOut = mAngularPID.update(headingErrRad);
            } else {
                // Exact arc angle: angle = 2 * asin(chord / (2*|radius|))
                // Using chordDist/radius (small-angle approximation) diverges for
                // large arcs — the exact formula matches the Arc utility.
                float halfAngle = std::asin(std::min(chordDist / (2.0f * radius), 1.0f));
                float arcSign = (req.targetRadius > 0) ? 1.0f : -1.0f;

                // Target heading tangential to arc at endpoint
                float targetHeadingRad = chordBearingRad + arcSign * (HALF_PI - halfAngle);
                float headingErrRad = angleDiffRad(pose.theta, targetHeadingRad);

                // Lateral error: how far off the arc
                float distErr = chordDist;
                lateralOut = mLateralPID.update(distErr);
                angularOut = mAngularPID.update(headingErrRad);
            }
            settled = chordDist < req.arcParams.targetTolerance;
            break;
        }
        case MotionType::Boomerang: {
            latLimit = req.boomerangParams.maxSpeed;
            // ================================================================
            // Boomerang controller: carrot-point guidance with lead decay
            // ================================================================
            // Phase 1: drive toward a "carrot point" ahead of the robot on
            //          the line to the target, producing a curved approach.
            // Phase 2: once close enough, align to the final heading.
            //
            // Reference: LemLib boomerang controller
            // ================================================================

            float dx = req.targetX - pose.x;
            float dy = req.targetY - pose.y;
            float distToTarget = std::sqrt(dx * dx + dy * dy);

            // Carrot point: interpolate between target and a lead point
            // Lead pulls the robot into a curved approach; lead decays with distance
            float lead = req.boomerangParams.lead;
            float leadDecay = req.boomerangParams.leadDecay;

            // Clamped lead: scales down as we get close to target
            float clampedLead = lead * distToTarget;
            if (clampedLead > 24.0f) clampedLead = 24.0f;

            // Carrot point: offset BEHIND the target along the target's FINAL
            // heading (not the robot→target bearing). This offset direction is
            // what makes the approach curve in to hit the target facing the
            // right way; using the bearing-to-target collapses the carrot onto
            // the straight line to the target and produces no curve at all.
            float carrotX = req.targetX - clampedLead * std::cos(req.targetHeadingRad);
            float carrotY = req.targetY - clampedLead * std::sin(req.targetHeadingRad);

            // Interpolate carrot toward target as distance decreases.
            // Guard lead == 0: distToTarget/(0*12) is NaN when the robot sits
            // exactly on the target (0/0) — NaN would flow into the PID and
            // then into the motor voltages.
            float t = (lead > 1e-6f)
                      ? std::min(distToTarget / (lead * 12.0f), 1.0f)
                      : 1.0f;
            carrotX = req.targetX + (carrotX - req.targetX) * (1.0f - t * leadDecay);
            carrotY = req.targetY + (carrotY - req.targetY) * (1.0f - t * leadDecay);

            // Drive toward carrot point
            float dCarrotX = carrotX - pose.x;
            float dCarrotY = carrotY - pose.y;
            float distToCarrot = std::sqrt(dCarrotX * dCarrotX + dCarrotY * dCarrotY);
            float carrotBearingRad = std::atan2(dCarrotY, dCarrotX);

            // Lateral error to carrot
            float lateralErr = distToCarrot;
            lateralOut = mLateralPID.update(lateralErr);

            // Heading error: blend between facing carrot and facing final heading
            float headingToCarrotRad = angleDiffRad(pose.theta, carrotBearingRad);
            float headingToFinalRad  = angleDiffRad(pose.theta, req.targetHeadingRad);

            // Weight shifts from carrot heading (far) to final heading (near)
            float headingWeight = std::min(distToTarget / 12.0f, 1.0f);
            float blendedHeadingErrRad = headingToCarrotRad * headingWeight +
                headingToFinalRad * (1.0f - headingWeight);

            angularOut = mAngularPID.update(blendedHeadingErrRad);

            // Speed scaling: slow down as we approach
            float speedScale = std::min(distToTarget / 6.0f, 1.0f);
            if (speedScale < req.boomerangParams.minSpeed / req.boomerangParams.maxSpeed) {
                speedScale = req.boomerangParams.minSpeed / req.boomerangParams.maxSpeed;
            }
            lateralOut *= speedScale;

            // Settled when close to target AND heading aligned
            bool distSettled = distToTarget < req.boomerangParams.targetTolerance;
            bool headingSettled = std::fabs(headingToFinalRad) < degToRad(req.boomerangParams.headingTolerance);
            settled = distSettled && headingSettled;

            break;
        }
        case MotionType::RAMSETE: {
            // ================================================================
            // RAMSETE controller: nonlinear SE(2) trajectory tracking
            // ================================================================
            // Standard unicycle tracking law (WPILib / De Luca convention):
            //   k  = 2ζ·√(ω_d² + b·v_d²)
            //   v  = v_d·cos(e_θ) + k·e_x                          [in/s]
            //   ω  = ω_d + b·v_d·sinc(e_θ)·e_y + k·e_θ             [rad/s]
            //
            // Everything is in real units: v/v_d in inches/second, ω/ω_d in
            // radians/second.  v_d comes from a velocity profile over the path
            // (accel/decel), NOT a constant.  Outputs are converted to per-side
            // wheel speeds and emitted directly (no lateral/angular mixing), so
            // the sign convention is unambiguous.
            //
            // `b` is taken in the conventional meter-based parameterization
            // (b≈2.0, ζ≈0.7 are good defaults) and scaled to inches here.
            // ================================================================

            const auto& path = req.ramsetePath;
            if (path.size() < 2) {
                settled = true;
                break;
            }

            float distToEnd = distanceToPoint(pose, path.back().x, path.back().y);

            // — Reference pose: closest point on the path, nudged forward by a
            //   small lookahead so the robot always aims slightly ahead. —
            int closestIdx = closestPathIndex(path, pose);
            int targetIdx = lookaheadIndex(path, pose, req.ramseteParams.lookaheadDist);
            if (distToEnd < req.ramseteParams.lookaheadDist) {
                targetIdx = static_cast<int>(path.size()) - 1;
            }
            const Pose& target = path[targetIdx];

            // — Desired forward speed v_d (in/s) from the velocity profile,
            //   sampled at the robot's arc-length progress along the path. —
            float progress = (closestIdx >= 0 &&
                              closestIdx < static_cast<int>(ramseteCumLen.size()))
                             ? ramseteCumLen[closestIdx] : 0.0f;
            float v_d = maxSpeedInPerSec * (req.ramseteParams.maxSpeed / 127.0f);
            if (!ramseteProfile.empty()) {
                // profile positions are monotonic — take the first sample at or
                // past our progress (its velocity is the target speed there).
                v_d = ramseteProfile.back().velocity;
                for (const auto& pt : ramseteProfile) {
                    if (pt.position >= progress) { v_d = pt.velocity; break; }
                }
            }

            // — Reference angular velocity ω_d = v_d · curvature (rad/s) —
            float w_d = 0.0f;
            if (targetIdx > 0 && targetIdx < static_cast<int>(path.size()) - 1) {
                const Pose& prev = path[targetIdx - 1];
                const Pose& next = path[targetIdx + 1];
                float segDist = distanceToPoint(prev, next.x, next.y);
                if (segDist > 1e-3f) {
                    w_d = v_d * angleDiffRad(prev.theta, next.theta) / segDist;
                }
            }

            // — Pose error in the robot frame —
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float e_x =  dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
            float e_y = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
            float e_theta = angleDiffRad(pose.theta, target.theta);

            // — RAMSETE gains + law (b scaled meters→inches) —
            float b = req.ramseteParams.b * (INCH_TO_METER * INCH_TO_METER);
            float zeta = req.ramseteParams.zeta;
            float k = 2.0f * zeta * std::sqrt(w_d * w_d + b * v_d * v_d);

            // sinc(e_θ) = sin(e_θ)/e_θ, → 1 as e_θ → 0 (avoids 0/0)
            float sinc = (std::fabs(e_theta) < 1e-4f)
                         ? 1.0f : std::sin(e_theta) / e_theta;

            float v = v_d * std::cos(e_theta) + k * e_x;             // in/s
            float w = w_d + b * v_d * sinc * e_y + k * e_theta;      // rad/s

            // — Convert (v, ω) to per-side wheel speeds and then motor units —
            //   left/right wheel linear speed = v ∓ ω·(trackWidth/2)  [in/s]
            //   CCW (ω>0, increasing heading) → right side faster, matching the
            //   library's standard-math heading convention.
            float halfTrack = mDrivetrain.trackWidth * 0.5f;
            float vLeft  = v - w * halfTrack;
            float vRight = v + w * halfTrack;
            float toMotor = (maxSpeedInPerSec > 1e-3f) ? (127.0f / maxSpeedInPerSec) : 0.0f;
            float maxOut = req.ramseteParams.maxSpeed;
            customLeft  = clamp(vLeft  * toMotor, -maxOut, maxOut);
            customRight = clamp(vRight * toMotor, -maxOut, maxOut);
            useCustomOutput = true;

            settled = distToEnd < req.ramseteParams.targetTolerance &&
                std::fabs(e_theta) < degToRad(req.ramseteParams.headingTolerance);

            break;
        }
        case MotionType::Swing: {
            angLimit = req.swingParams.maxSpeed;
            // Swing turn: pivot around one stationary wheel set
            float headingErrRad = angleDiffRad(pose.theta, req.targetHeadingRad);

            // Determine direction
            TurnDirection dir = req.swingParams.direction;
            if (dir == TurnDirection::CW) {
                headingErrRad = -std::fabs(headingErrRad);
            } else if (dir == TurnDirection::CCW) {
                headingErrRad = std::fabs(headingErrRad);
            }
            // SHORTEST: keep natural sign

            angularOut = mAngularPID.update(headingErrRad);
            lateralOut = 0;

            // Only one side moves — store in custom output vars
            if (req.swingSide == SwingSide::Left) {
                // Left stationary, right turns.  For CCW (+angularOut),
                // the right motor must go backward → negate angularOut.
                customLeft = 0;
                customRight = clamp(-angularOut, -127.0f, 127.0f);
            } else {
                // Right stationary, left turns.  For CCW (+angularOut),
                // the left motor goes forward.
                customLeft = clamp(angularOut, -127.0f, 127.0f);
                customRight = 0;
            }
            useCustomOutput = true;

            settled = std::fabs(headingErrRad) < degToRad(req.swingParams.targetTolerance);
            break;
        }
        default:
            break;
        }

        // Clamp outputs using the motion-type-specific limits
        lateralOut = clamp(lateralOut, -latLimit, latLimit);
        angularOut = clamp(angularOut, -angLimit, angLimit);

        // Compute motor outputs: tank-style mix, or direct (Swing / RAMSETE)
        {
        float left, right;
        if (useCustomOutput) {
            left  = customLeft;
            right = customRight;
        } else {
            left  = clamp(lateralOut + angularOut, -127.0f, 127.0f);
            right = clamp(lateralOut - angularOut, -127.0f, 127.0f);
        }

        if (!mDryRun) {
            mDrivetrain.setVoltage(left, right);
        } else {
            // — Dry-run: integrate motor outputs into synthetic pose —
            // Convert voltage outputs → linear/angular velocity → pose update
            float leftFrac  = left / 127.0f;
            float rightFrac = right / 127.0f;

            float linearVel  = (leftFrac + rightFrac) * 0.5f * maxSpeedInPerSec;
            float angularVel = (leftFrac - rightFrac) * maxSpeedInPerSec /
                mDrivetrain.trackWidth;

            float dt = 0.01f;  // 10ms loop period
            float dThetaRad = angularVel * dt;
            float halfDThetaRad = dThetaRad * 0.5f;

            float dX, dY;
            if (std::fabs(dThetaRad) < 1e-6f) {
                dX = linearVel * std::cos(mDryRunPose.theta) * dt;
                dY = linearVel * std::sin(mDryRunPose.theta) * dt;
            } else {
                // Arc approximation
                float midHeadingRad = mDryRunPose.theta + halfDThetaRad;
                float arcDist = linearVel * dt;
                dX = arcDist * std::cos(midHeadingRad);
                dY = arcDist * std::sin(midHeadingRad);
            }

            mDryRunPose.x += dX;
            mDryRunPose.y += dY;
            mDryRunPose.theta += dThetaRad;
            mDryRunPose.theta = wrapRad(mDryRunPose.theta);

            mDryRunPath.push_back(mDryRunPose);
        }
        }

        // Timeout: abort if motion exceeds the requested duration.
        // Dry-run compares against SIMULATED time so previews keep the same
        // timeout semantics as the real run despite being fast-forwarded.
        float elapsedMs = mDryRun ? simTimeMs
                                  : static_cast<float>(pros::millis() - req.startTime);
        if (req.timeout > 0 && elapsedMs > req.timeout) {
            settled = true;
        }

        if (settled) break;

        if (!mDryRun) {
            pros::delay(10);
        } else {
            simTimeMs += 10.0f;                       // one simulated tick
            if (++dryRunIters >= DRYRUN_MAX_ITERS) break;
            if ((dryRunIters & 31) == 0) pros::delay(1);  // yield CPU periodically
        }
    }

    endMotion();
}

// ========================================================================
// Movement Commands (public API)
// ========================================================================

// Every movement command builds a MotionRequest and hands it to enqueueMotion,
// which runs it on the single motion task (blocking waits there for completion).
// This keeps the motors, PIDs, and mCurrentMotion single-threaded.

void Chassis::moveDistance(float target, float timeout, const MoveDistanceParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Distance;
    req.targetDistance = target;     // store raw distance — target X/Y computed at exec time
    req.moveDistParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::moveToPoint(float x, float y, float timeout, const MoveToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Point;
    req.targetX = x;
    req.targetY = y;
    req.movePointParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::turnToHeading(float thetaDeg, float timeout, const TurntoHeadingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Heading;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.turnHeadingParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::turnToPoint(float x, float y, float timeout, const TurnToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::TurnPoint;
    req.targetX = x;
    req.targetY = y;
    req.turnPointParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::moveArc(float x, float y, float radius, float timeout, const ArcParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Arc;
    req.targetX = x;
    req.targetY = y;
    req.targetRadius = radius;
    req.arcParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::moveBoomerang(float x, float y, float thetaDeg, float timeout, const BoomerangParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Boomerang;
    req.targetX = x;
    req.targetY = y;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.boomerangParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::RAMSETE;
    req.ramsetePath = path;
    req.ramseteParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::swingToHeading(float thetaDeg, SwingSide side, float timeout, const SwingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Swing;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.swingSide = side;
    req.swingParams = params;
    req.timeout = timeout;
    enqueueMotion(req, async);
}

void Chassis::enqueueMotion(MotionRequest& req, bool async) {
    ensureMotionTask();

    // Serialize enqueues: wait until no motion is queued so we keep the
    // "at most one pending" invariant and callers get sequential ordering.
    mQueueMutex.lock();
    while (!mMotionQueue.empty()) {
        mQueueMutex.unlock();
        if (mMotionCancelled) return;   // don't stack behind a cancel-in-progress
        pros::delay(5);
        mQueueMutex.lock();
    }
    req.seq = ++mMotionSeqCounter;
    mMotionQueue.push(req);
    mQueueMutex.unlock();
    mMotionTask->notify();

    if (!async) {
        // Block until the motion task finishes THIS motion (by sequence id).
        // A plain "queue empty && !running" check can return in the gap between
        // the task popping the request and marking it running, so we wait on a
        // completion counter that only advances after the motion actually ends.
        uint32_t mySeq = req.seq;
        while (static_cast<int32_t>(mCompletedSeq.load() - mySeq) < 0) {
            if (mMotionCancelled) break;
            pros::delay(5);
        }
    }
}

// ========================================================================
// Motion Status
// ========================================================================

bool Chassis::isSettled() const {
    // Settled once the task has completed every enqueued motion. Using the
    // sequence ids also covers a motion that is queued but not yet running.
    return static_cast<int32_t>(mCompletedSeq.load() - mMotionSeqCounter.load()) >= 0;
}

void Chassis::cancelMotion() {
    mMotionCancelled = true;          // signal current motion loop to stop

    // Clear all pending queued motions
    mQueueMutex.lock();
    while (!mMotionQueue.empty()) mMotionQueue.pop();
    mQueueMutex.unlock();

    // Wake the persistent task so it sees the cancel
    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }

    // Wait for the current motion to actually stop
    while (mMotionRunning) {
        pros::delay(5);
    }

    // Release any blocking caller: every motion enqueued so far is now
    // resolved (either it ran, or it was dropped from the queue above).
    mCompletedSeq.store(mMotionSeqCounter.load());

    // Reset cancel flag — without this, subsequent async motions are
    // silently discarded because runMotionTask() checks mMotionCancelled
    // after dequeue and skips execution if it's still true.
    mMotionCancelled = false;

    // Safety: ensure motors are off (defense in depth)
    mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    // Wait until the motion task has completed every enqueued motion. Comparing
    // sequence ids (rather than "queue empty && !running") closes the gap where
    // a just-enqueued motion is popped but not yet marked running.
    while (static_cast<int32_t>(mCompletedSeq.load() - mMotionSeqCounter.load()) < 0) {
        pros::delay(10);
    }
}

void Chassis::waitUntilDist(float dist) {
    // In dry-run the real pose never moves — waiting would hang the preview.
    if (mDryRun) return;
    Pose start = mOdom.getPose();
    // The background odometry task keeps the pose fresh — no update() here
    // (calling it from this task too would race and consume MCL's deltas).
    while (distanceToPoint(mOdom.getPose(), start.x, start.y) < dist) {
        pros::delay(10);
    }
}

void Chassis::ensureMotionTask() {
    if (mMotionTask == nullptr) {
        mTaskShouldStop = false;
        mMotionTask = new pros::Task([this]() { runMotionTask(); });
    }
}

// ========================================================================
// Tracking Access
// ========================================================================

OdomTracking& Chassis::odom() { return mOdom; }
RclTracking& Chassis::rcl()  { return mRcl; }
MclTracking& Chassis::mcl()  { return mMcl; }

void Chassis::startTracking() {
    if (mConfig.useRclTracking) mRcl.startTracking();
    if (mConfig.useMclTracking) mMcl.startTracking();
}

void Chassis::stopTracking() {
    mRcl.stopTracking();
    mMcl.stopTracking();
}

// ========================================================================
// Tuning
// ========================================================================

void Chassis::setLateralGains(const PIDGains& gains) {
    mLateralPID.setGains(gains);
}

void Chassis::setAngularGains(const PIDGains& gains) {
    mAngularPID.setGains(gains);
}

void Chassis::setThrottleCurve(const DriveCurve& curve) {
    mThrottleCurve = curve;
}

void Chassis::setSteerCurve(const DriveCurve& curve) {
    mSteerCurve = curve;
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    mDrivetrain.setBrakeMode(mode);
}

void Chassis::setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors) {
    mConfig.distanceSensors = sensors;
    mMcl.setDistanceSensors(sensors);
    mRcl.configureSensors(sensors, mConfig.sensorMounts);
}

// ========================================================================
// Dry-run mode
// ========================================================================

void Chassis::setDryRun(bool enabled) {
    mDryRun = enabled;
    if (enabled) {
        mDryRunPath.clear();
        mDryRunStartPose = mOdom.getPose();
        // Initialize the simulated pose too — without this, the simulation
        // continued from wherever the PREVIOUS dry-run left off.
        mDryRunPose = mDryRunStartPose;
        mDryRunPath.push_back(mDryRunStartPose);
    }
}

bool Chassis::isDryRun() const {
    return mDryRun;
}

const std::vector<Pose>& Chassis::dryRunPath() const {
    return mDryRunPath;
}

void Chassis::resetDryRunPath() {
    mDryRunPath.clear();
}

}  // namespace FBLIB
