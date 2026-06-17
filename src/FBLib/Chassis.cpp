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
      horizontalDrift(horizontalDrift) {}

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

    double sum = 0.0;
    int count = 0;
    if (leftMotors != nullptr) {
        for (double pos : leftMotors->get_position_all()) { sum += pos; count++; }
    }
    if (rightMotors != nullptr) {
        for (double pos : rightMotors->get_position_all()) { sum += pos; count++; }
    }
    return (count > 0) ? static_cast<float>(sum / count) : 0.0f;
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
}

Chassis::~Chassis() {
    mTaskShouldStop = true;
    if (mMotionTask != nullptr) {
        mMotionTask->notify();       // wake from notify_take so it can exit
        pros::delay(20);             // give it a moment to exit cleanly
        mMotionTask->remove();
        delete mMotionTask;
        mMotionTask = nullptr;
    }
}

// ========================================================================
// Calibration & Pose
// ========================================================================

void Chassis::calibrate() {
    mOdom.calibrate();
}

void Chassis::setPose(const Pose& pose) {
    mOdom.setPose(pose);
    // Propagate to MCL so particles are reinitialized around the new pose.
    // Without this, MCL particles stay at the old pose when the user changes
    // the chassis pose (e.g. during competition_initialize).
    mMcl.setPose(pose);
    mRcl.setRclPose(pose);
    // If we're in dry-run mode, re-baseline the simulation so the path
    // starts from wherever the auton says it should start.
    if (mDryRun) {
        mDryRunStartPose = pose;
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
    }
}

void Chassis::setPosition(float x, float y) {
    Pose current = mOdom.getPose();
    Pose newPose{x, y, current.theta};
    mOdom.setPose(newPose);
    // Propagate to tracking systems so MCL particles stay centered and RCL
    // estimate stays synchronized.
    mMcl.setPose(newPose);
    mRcl.setRclPose(newPose);
}

void Chassis::setHeading(float thetaDeg) {
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
    float l = mThrottleCurve.apply(left);
    float r = mThrottleCurve.apply(right);
    mDrivetrain.setVoltage(l, r);
}

void Chassis::arcade(float throttle, float steer) {
    if (mDryRun) return;
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

        // Drain all queued motions without dropping mMotionRunning between them
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

            // If cancel() was called between motions, honor it
            if (mMotionCancelled) break;

            // Wait for any blocking motion (running on calling thread) to finish
            while (mMotionRunning) {
                pros::delay(5);
                if (mMotionCancelled) break;
            }
            if (mMotionCancelled) break;

            mCurrentMotion = req;
            executeCurrentMotion();
        }
    }
}

void Chassis::executeCurrentMotion() {
    auto& req = mCurrentMotion;

    // Timestamp when execution actually begins (not when enqueued)
    req.startTime = pros::millis();

    // Dry-run: snapshot pose at motion start so path preview originates here
    if (mDryRun) {
        mDryRunStartPose = mOdom.getPose();
        mDryRunPath.clear();
        mDryRunPath.push_back(mDryRunStartPose);
        mDryRunPose = mDryRunStartPose;
    }

    // Precompute max linear speed for dry-run integration
    // v_max = rpm / 60 * wheel_diameter * PI  (inches/second)
    float maxSpeedInPerSec = mDrivetrain.rpm / 60.0f *
        mDrivetrain.wheelDiameter * PI;

    mMotionCancelled = false;
    startMotion();

    while (mMotionRunning && !mMotionCancelled) {
        mOdom.update();
        Pose pose = mDryRun ? mDryRunPose : mOdom.getPose();

        float lateralOut = 0.0f, angularOut = 0.0f;
        float swingLeft = 0.0f, swingRight = 0.0f;
        bool settled = false;
        bool useCustomOutput = false;  // Swing sets its own motor outputs
        float latLimit = 127.0f;       // max lateral output (±)
        float angLimit = 127.0f;       // max angular output (±)

        switch (req.type) {
        case MotionType::Distance: {
            latLimit = req.moveDistParams.maxSpeed;
            // Recompute absolute field target at execution time.
            // This avoids stale targets when the motion is queued behind
            // a turn — the target is always relative to the robot's
            // pose when execution actually begins.
            float dir = req.moveDistParams.forwards ? 1.0f : -1.0f;
            float targetX = pose.x + req.targetDistance * dir * std::cos(pose.theta);
            float targetY = pose.y + req.targetDistance * dir * std::sin(pose.theta);
            float distErr = distanceToPoint(pose, targetX, targetY);
            lateralOut = mLateralPID.update(distErr);

            float targetBearingRad = std::atan2(targetY - pose.y, targetX - pose.x);
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
            float bearingToTargetRad = std::atan2(dy, dx);

            // Carrot point: interpolate between target and a lead point
            // Lead pulls the robot into a curved approach; lead decays with distance
            float lead = req.boomerangParams.lead;
            float leadDecay = req.boomerangParams.leadDecay;

            // Clamped lead: scales down as we get close to target
            float clampedLead = lead * distToTarget;
            if (clampedLead > 24.0f) clampedLead = 24.0f;

            // Carrot point: ahead of target along approach bearing
            float carrotX = req.targetX - clampedLead * std::cos(bearingToTargetRad);
            float carrotY = req.targetY - clampedLead * std::sin(bearingToTargetRad);

            // Interpolate carrot toward target as distance decreases
            float t = std::min(distToTarget / (lead * 12.0f), 1.0f);
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
            latLimit = req.ramseteParams.maxSpeed;
            // ================================================================
            // RAMSETE controller: nonlinear SE(2) trajectory tracking
            // ================================================================
            // v = v_d * cos(theta_err) + k_x * x_err
            // w = w_d + k_y * v_d * y_err + k_theta * v_d * sin(theta_err)
            //
            // Combined with pure-pursuit for lookahead target selection.
            // Reference: "Control of Wheeled Mobile Robots" (De Luca et al.)
            // ================================================================

            const auto& path = req.ramsetePath;
            if (path.empty()) {
                settled = true;
                break;
            }

            // Find the lookahead point on the path (pure pursuit).
            // Must find the CLOSEST point first, then look ahead from there —
            // starting from index 0 would target early points when the robot
            // is near the end of the path, causing it to turn around.
            int targetIdx = lookaheadIndex(path, pose, req.ramseteParams.lookaheadDist);

            // If we're closer to the end than lookahead, target the final point
            float distToEnd = distanceToPoint(pose, path.back().x, path.back().y);
            if (distToEnd < req.ramseteParams.lookaheadDist) {
                targetIdx = static_cast<int>(path.size()) - 1;
            }

            const Pose& target = path[targetIdx];

            // Transform target pose into robot frame
            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float thetaErrRad = angleDiffRad(pose.theta, target.theta);

            float xErr =  dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
            float yErr = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);

            // Desired velocities (approximate from path geometry)
            float v_d = req.ramseteParams.maxSpeed;
            float w_d = 0.0f;

            // Compute curvature-based angular velocity if we have adjacent points
            if (targetIdx > 0 && targetIdx < static_cast<int>(path.size()) - 1) {
                const Pose& prev = path[targetIdx - 1];
                const Pose& next = path[targetIdx + 1];
                float segDist = distanceToPoint(prev, next.x, next.y);
                if (segDist > 0.01f) {
                    w_d = v_d * angleDiffRad(prev.theta, next.theta) / segDist;
                }
            }

            // RAMSETE control law
            float b = req.ramseteParams.b;
            float zeta = req.ramseteParams.zeta;

            // Nonlinear feedback gains
            float k_x = 2.0f * zeta * std::sqrt(w_d * w_d + b * v_d * v_d);
            float k_y = b * v_d;
            float k_theta = k_x;

            // Control outputs
            float v = v_d * std::cos(thetaErrRad) + k_x * xErr;
            float w = w_d + k_y * v_d * yErr + k_theta * v_d * std::sin(thetaErrRad);

            // Scale to motor outputs
            lateralOut = clamp(v / req.ramseteParams.maxSpeed * 127.0f, -127.0f, 127.0f);
            angularOut = clamp(w * 20.0f, -127.0f, 127.0f);  // empirical scaling

            // Speed limiting
            float speedScale = std::min(distToEnd / 3.0f, 1.0f);
            if (speedScale < req.ramseteParams.minSpeed / req.ramseteParams.maxSpeed) {
                speedScale = req.ramseteParams.minSpeed / req.ramseteParams.maxSpeed;
            }
            lateralOut *= speedScale;

            settled = distToEnd < req.ramseteParams.targetTolerance &&
                std::fabs(thetaErrRad) < degToRad(req.ramseteParams.headingTolerance);

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
                swingLeft = 0;
                swingRight = clamp(-angularOut, -127.0f, 127.0f);
            } else {
                // Right stationary, left turns.  For CCW (+angularOut),
                // the left motor goes forward.
                swingLeft = clamp(angularOut, -127.0f, 127.0f);
                swingRight = 0;
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

        // Compute motor outputs: tank-style or custom (Swing)
        {
        float left, right;
        if (useCustomOutput) {
            left  = swingLeft;
            right = swingRight;
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

        // Timeout: abort if motion exceeds the requested duration
        if (req.timeout > 0 && (pros::millis() - req.startTime) > req.timeout) {
            settled = true;
        }

        if (settled) break;

        pros::delay(10);
    }

    endMotion();
}

// ========================================================================
// Movement Commands (public API)
// ========================================================================

void Chassis::moveDistance(float target, float timeout, const MoveDistanceParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Distance;
    req.targetDistance = target;     // store raw distance — target X/Y computed at exec time
    req.moveDistParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::moveToPoint(float x, float y, float timeout, const MoveToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Point;
    req.targetX = x;
    req.targetY = y;
    req.movePointParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::turnToHeading(float thetaDeg, float timeout, const TurntoHeadingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Heading;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.turnHeadingParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::turnToPoint(float x, float y, float timeout, const TurnToPointParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::TurnPoint;
    req.targetX = x;
    req.targetY = y;
    req.turnPointParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::moveArc(float x, float y, float radius, float timeout, const ArcParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Arc;
    req.targetX = x;
    req.targetY = y;
    req.targetRadius = radius;
    req.arcParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::moveBoomerang(float x, float y, float thetaDeg, float timeout, const BoomerangParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Boomerang;
    req.targetX = x;
    req.targetY = y;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.boomerangParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::RAMSETE;
    req.ramsetePath = path;
    req.ramseteParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

void Chassis::swingToHeading(float thetaDeg, SwingSide side, float timeout, const SwingParams& params, bool async) {
    MotionRequest req;
    req.type = MotionType::Swing;
    req.targetHeadingRad = vexToStdRad(thetaDeg);
    req.swingSide = side;
    req.swingParams = params;
    req.timeout = timeout;

    if (async) {
        ensureMotionTask();
        mQueueMutex.lock();
        while (!mMotionQueue.empty()) {   // at most one queued motion
            mQueueMutex.unlock();
            pros::delay(5);
            mQueueMutex.lock();
        }
        mMotionQueue.push(req);
        mQueueMutex.unlock();
        mMotionTask->notify();
    } else {
        waitUntilSettled();    // wait for running/queued motions to finish
        mCurrentMotion = req;
        executeCurrentMotion();
    }
}

// ========================================================================
// Motion Status
// ========================================================================

bool Chassis::isSettled() const {
    return !mMotionRunning;
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

    // Reset cancel flag — without this, subsequent async motions are
    // silently discarded because runMotionTask() checks mMotionCancelled
    // after dequeue and skips execution if it's still true.
    mMotionCancelled = false;

    // Safety: ensure motors are off (defense in depth)
    mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    while (true) {
        bool queueEmpty;
        mQueueMutex.lock();
        queueEmpty = mMotionQueue.empty();
        mQueueMutex.unlock();
        if (queueEmpty && !mMotionRunning) break;
        pros::delay(10);
    }
}

void Chassis::waitUntilDist(float dist) {
    float startX = mOdom.getPose().x;
    float startY = mOdom.getPose().y;
    while (distanceToPoint(mOdom.getPose(), startX, startY) < dist) {
        // Keep odometry fresh — without this, the pose never changes
        // and the loop spins forever if no background task is running.
        mOdom.update();
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
