#include "FBLib/Chassis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

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
    // set_brake_mode() on a MotorGroup only changes ONE motor (index 0);
    // the _all variant is needed to reach every motor in the group.
    if (leftMotors != nullptr) leftMotors->set_brake_mode_all(mode);
    if (rightMotors != nullptr) rightMotors->set_brake_mode_all(mode);
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

float Drivetrain::maxSpeedInPerSec() const {
    return rpm / 60.0f * wheelDiameter * PI;
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

namespace {
constexpr uint32_t MOTION_PERIOD_MS = 10;
constexpr float MOTION_PERIOD_S = MOTION_PERIOD_MS / 1000.0f;
constexpr int DRYRUN_MAX_ITERS = 60000;       // 10 simulated minutes per motion
constexpr float STALL_PROGRESS_EPS = 0.05f;   // inches of wheel travel that count as progress
}  // namespace

Chassis::Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
                 const ChassisConfig& config)
    : mDrivetrain(drivetrain),
      mConfig(config),
      mOdom(sensors),
      mRcl(mOdom, config.rclConfig),
      mMcl(mOdom, config.distanceSensors, Pose{}, config.mclConfig),
      mLateralPID(config.lateralGains, config.lateralWindupRange, config.lateralFlipReset),
      mAngularPID(config.angularGains, config.angularWindupRange, config.angularFlipReset),
      mLateralGains(config.lateralGains),
      mAngularGains(config.angularGains),
      mThrottleCurve(config.throttleCurve),
      mSteerCurve(config.steerCurve)
{
    // Apply user-configured sensor mount offsets (robot-specific)
    mMcl.setSensorMounts(config.sensorMounts);

    // Auto-configure RCL sensors from config arrays.
    // Null distance sensor entries are automatically disabled.
    mRcl.configureSensors(config.distanceSensors, config.sensorMounts);

    // Start the persistent motion task (created once, reused for all motions)
    ensureMotionTask();

    // Start the background odometry task — the SOLE caller of mOdom.update().
    // Centralizing updates in one task avoids racing odometry state and lets
    // MCL consume accumulated deltas without losing any ticks. It also keeps
    // the pose fresh during driver control, when no motion is running.
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
    // Stop the motion task: end any running motion and wake it from notify_take
    mTaskShouldStop = true;
    mCancelGeneration++;
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
    // Re-centre MCL/RCL on the re-zeroed pose.
    Pose pose = mOdom.getPose();
    mMcl.setPose(pose);
    mRcl.setRclPose(pose);
}

void Chassis::setPose(const Pose& pose) {
    // Previewing: only re-baseline the SIMULATION. Auton functions call
    // setPose() at their start, and touching real odometry/MCL/RCL here would
    // teleport the robot's actual localization every time a preview runs.
    if (previewing()) {
        std::lock_guard<pros::Mutex> lock(mDryRunMutex);
        mDryRunPose = pose;
        mDryRunPath.clear();
        mDryRunPath.push_back(pose);
        return;
    }
    mOdom.setPose(pose);
    // Propagate to MCL so particles are reinitialized around the new pose,
    // and keep the RCL estimate synchronized.
    Pose applied = mOdom.getPose();
    mMcl.setPose(applied);
    mRcl.setRclPose(applied);
}

void Chassis::setPosition(float x, float y) {
    if (previewing()) {
        std::lock_guard<pros::Mutex> lock(mDryRunMutex);
        mDryRunPose.x = x;
        mDryRunPose.y = y;
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    mOdom.setPosition(x, y);
    Pose applied = mOdom.getPose();
    mMcl.setPose(applied);
    mRcl.setRclPose(applied);
}

void Chassis::setHeading(float thetaDeg) {
    if (previewing()) {
        std::lock_guard<pros::Mutex> lock(mDryRunMutex);
        mDryRunPose.theta = vexToStdRad(thetaDeg);
        mDryRunPath.push_back(mDryRunPose);
        return;
    }
    mOdom.setHeading(vexToStdRad(thetaDeg));
    Pose applied = mOdom.getPose();
    mMcl.setPose(applied);
    mRcl.setRclPose(applied);
}

Pose Chassis::getPose() const {
    return previewing() ? simulatedPose() : mOdom.getPose();
}

// ========================================================================
// Driver Control
// ========================================================================
//
// Driver input is ignored while a real motion is queued or running (so an
// async macro isn't fought by the sticks) and from a previewing task.

void Chassis::tank(float left, float right) {
    if (previewing() || mPendingRealMotions.load() > 0) return;
    float l = mThrottleCurve.apply(left);
    float r = mThrottleCurve.apply(right);
    mDrivetrain.setVoltage(l, r);
}

void Chassis::arcade(float throttle, float steer) {
    if (previewing() || mPendingRealMotions.load() > 0) return;
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
    if (previewing() || mPendingRealMotions.load() > 0) return;
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
// Movement Commands (public API) — build the Motion, hand it to the queue
// ========================================================================

void Chassis::moveDistance(float target, float timeout, const MoveDistanceParams& params, bool async) {
    enqueueMotion(std::make_unique<MoveDistanceMotion>(target, params), timeout, async);
}

void Chassis::moveToPoint(float x, float y, float timeout, const MoveToPointParams& params, bool async) {
    enqueueMotion(std::make_unique<MoveToPointMotion>(x, y, params), timeout, async);
}

void Chassis::turnToHeading(float thetaDeg, float timeout, const TurntoHeadingParams& params, bool async) {
    enqueueMotion(std::make_unique<TurnToHeadingMotion>(vexToStdRad(thetaDeg), params), timeout, async);
}

void Chassis::turnToPoint(float x, float y, float timeout, const TurnToPointParams& params, bool async) {
    enqueueMotion(std::make_unique<TurnToPointMotion>(x, y, params), timeout, async);
}

void Chassis::moveArc(float x, float y, float radius, float timeout, const ArcParams& params, bool async) {
    enqueueMotion(std::make_unique<ArcMotion>(x, y, radius, params), timeout, async);
}

void Chassis::moveBoomerang(float x, float y, float thetaDeg, float timeout, const BoomerangParams& params, bool async) {
    enqueueMotion(std::make_unique<BoomerangMotion>(x, y, vexToStdRad(thetaDeg), params), timeout, async);
}

void Chassis::moveRAMSETE(const std::vector<Pose>& path, float timeout, const RAMSETEParams& params, bool async) {
    enqueueMotion(std::make_unique<RamseteMotion>(path, params), timeout, async);
}

void Chassis::swingToHeading(float thetaDeg, SwingSide side, float timeout, const SwingParams& params, bool async) {
    enqueueMotion(std::make_unique<SwingMotion>(vexToStdRad(thetaDeg), side, params), timeout, async);
}

void Chassis::runMotion(std::unique_ptr<Motion> motion, float timeout, bool async) {
    enqueueMotion(std::move(motion), timeout, async);
}

// ========================================================================
// Motion queue and execution
// ========================================================================

void Chassis::enqueueMotion(std::unique_ptr<Motion> motion, float timeout, bool async) {
    if (!motion) return;
    ensureMotionTask();

    const bool simulate = previewing();
    const uint32_t generation = mCancelGeneration.load();
    uint32_t seq;

    // Serialize enqueues: wait until no motion is queued so we keep the
    // "at most one pending" invariant and callers get sequential ordering.
    mQueueMutex.lock();
    while (!mMotionQueue.empty()) {
        mQueueMutex.unlock();
        if (mCancelGeneration.load() != generation) return;  // cancelled while waiting
        pros::delay(5);
        mQueueMutex.lock();
    }
    MotionRequest req;
    req.seq = seq = ++mMotionSeqCounter;
    req.generation = generation;
    req.dryRun = simulate;
    req.timeout = timeout;
    req.motion = std::move(motion);
    if (simulate) mLastDryRunSeq = seq;
    else mPendingRealMotions++;
    mMotionQueue.push(std::move(req));
    mQueueMutex.unlock();
    mMotionTask->notify();

    if (!async) {
        // Block until the motion task finishes THIS motion (by sequence id).
        // Every request is either executed or skipped (cancelled), so the
        // completion counter always reaches it.
        while (static_cast<int32_t>(mCompletedSeq.load() - seq) < 0) {
            pros::delay(simulate ? 1 : 5);
        }
    }
}

void Chassis::runMotionTask() {
    while (!mTaskShouldStop) {
        // Block until work arrives (FreeRTOS task notification)
        pros::Task::notify_take(true, TIMEOUT_MAX);
        if (mTaskShouldStop) break;

        // Drain all queued motions. This task is the SOLE executor of motions,
        // so nothing else touches the PIDs, the motors during a motion, or the
        // simulation.
        while (!mTaskShouldStop) {
            MotionRequest req;
            {
                std::lock_guard<pros::Mutex> lock(mQueueMutex);
                if (mMotionQueue.empty()) break;
                req = std::move(mMotionQueue.front());
                mMotionQueue.pop();
            }

            // A request from before the last cancelMotion() is skipped, but its
            // sequence id is still published so a blocking caller is released.
            if (req.generation == mCancelGeneration.load()) {
                executeMotion(req);
            }
            if (!req.dryRun) mPendingRealMotions--;
            mCompletedSeq.store(req.seq);
        }
    }
}

void Chassis::executeMotion(MotionRequest& req) {
    const bool simulate = req.dryRun;

    // Start every motion with clean controllers and the latest gains.
    {
        std::lock_guard<pros::Mutex> lock(mGainsMutex);
        mLateralPID.setGains(mLateralGains);
        mAngularPID.setGains(mAngularGains);
    }
    mLateralPID.reset();
    mAngularPID.reset();

    MotionContext ctx{mLateralPID, mAngularPID, mDrivetrain.trackWidth, mDrivetrain.maxSpeedInPerSec()};
    req.motion->start(simulate ? simulatedPose() : mOdom.getPose(), ctx);

    const uint32_t startMs = pros::millis();
    uint32_t lastTickMs = startMs;
    float elapsedMs = 0.0f;           // real time, or simulated time when previewing
    float bestRemaining = std::numeric_limits<float>::infinity();
    float lastProgressMs = 0.0f;

    // Previews run FAST-FORWARDED: simulated time advances one period per
    // iteration without sleeping, so a 15 s auton previews in milliseconds
    // instead of freezing the caller (e.g. an LVGL callback). The iteration
    // cap guarantees termination even if a simulated motion never settles.
    for (int iteration = 0; req.generation == mCancelGeneration.load() && !mTaskShouldStop; iteration++) {
        float dt = MOTION_PERIOD_S;
        if (!simulate && iteration > 0) {
            uint32_t now = pros::millis();
            dt = (now - lastTickMs) / 1000.0f;
            lastTickMs = now;
            elapsedMs = static_cast<float>(now - startMs);
        }

        // Odometry freshness comes from the background odometry task.
        Pose pose = simulate ? simulatedPose() : mOdom.getPose();
        MotionOutput out = req.motion->update(pose, dt, ctx);

        if (simulate) {
            stepSimulation(out.left, out.right, MOTION_PERIOD_S);
        } else {
            mDrivetrain.setVoltage(out.left, out.right);
        }

        if (out.done) break;
        if (req.timeout > 0 && elapsedMs >= req.timeout) break;

        // Stall exit: remaining travel hasn't improved for stallTimeoutMs.
        // Motions that don't report progress (remaining < 0) are exempt.
        if (out.remaining >= 0.0f) {
            if (out.remaining < bestRemaining - STALL_PROGRESS_EPS) {
                bestRemaining = out.remaining;
                lastProgressMs = elapsedMs;
            }
            if (mConfig.stallTimeoutMs > 0.0f && elapsedMs - lastProgressMs > mConfig.stallTimeoutMs) break;
        }

        if (simulate) {
            elapsedMs += MOTION_PERIOD_MS;
            if (iteration + 1 >= DRYRUN_MAX_ITERS) break;
            if ((iteration & 31) == 31) pros::delay(1);  // yield CPU periodically
        } else {
            pros::delay(MOTION_PERIOD_MS);
        }
    }

    if (!simulate) mDrivetrain.setVoltage(0, 0);
    mLateralPID.reset();
    mAngularPID.reset();
}

void Chassis::ensureMotionTask() {
    if (mMotionTask == nullptr) {
        mTaskShouldStop = false;
        mMotionTask = new pros::Task([this]() { runMotionTask(); });
    }
}

// ========================================================================
// Motion Status
// ========================================================================

bool Chassis::isSettled() const {
    // A previewing task waits for the simulated motions it issued (motions run
    // in order, so the newest one finishing means all of them have). Using the
    // sequence id also covers a motion that is queued but not yet running.
    if (previewing()) {
        return static_cast<int32_t>(mCompletedSeq.load() - mLastDryRunSeq.load()) >= 0;
    }
    // Everyone else sees only the real robot: a preview running in another
    // task must not look like the drive is busy.
    return mPendingRealMotions.load() == 0;
}

void Chassis::cancelMotion() {
    // Everything issued before this point belongs to the old generation: the
    // running motion stops at its next tick and queued ones are skipped.
    // (A generation counter instead of a cancel flag: a flag that has to be
    // reset again races with the task picking up the next motion.)
    const uint32_t issued = mMotionSeqCounter.load();
    mCancelGeneration++;
    if (mMotionTask != nullptr) {
        mMotionTask->notify();
    }
    while (static_cast<int32_t>(mCompletedSeq.load() - issued) < 0) {
        pros::delay(5);
    }

    // Safety: ensure motors are off (defense in depth)
    if (!previewing()) mDrivetrain.setVoltage(0, 0);
}

void Chassis::waitUntilSettled() {
    while (!isSettled()) {
        pros::delay(previewing() ? 1 : 10);
    }
}

void Chassis::waitUntilDist(float dist) {
    // getPose() follows the simulated pose while previewing. Stop waiting when
    // the motion ends — it may finish (or time out) short of `dist`.
    Pose start = getPose();
    const bool simulate = previewing();
    while (!isSettled() && distanceToPoint(getPose(), start.x, start.y) < dist) {
        pros::delay(simulate ? 1 : 10);
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
    std::lock_guard<pros::Mutex> lock(mGainsMutex);
    mLateralGains = gains;
}

void Chassis::setAngularGains(const PIDGains& gains) {
    std::lock_guard<pros::Mutex> lock(mGainsMutex);
    mAngularGains = gains;
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

bool Chassis::previewing() const {
    pros::task_t task = mDryRunTask.load();
    return task != nullptr && task == pros::c::task_get_current();
}

Pose Chassis::simulatedPose() const {
    std::lock_guard<pros::Mutex> lock(mDryRunMutex);
    return mDryRunPose;
}

void Chassis::stepSimulation(float left, float right, float dt) {
    // Ideal tank drive: v = (vL + vR) / 2, omega = (vR - vL) / trackWidth.
    float vmax = mDrivetrain.maxSpeedInPerSec();
    float vLeft = clamp(left, -127.0f, 127.0f) / 127.0f * vmax;
    float vRight = clamp(right, -127.0f, 127.0f) / 127.0f * vmax;
    float v = 0.5f * (vLeft + vRight);
    float omega = (mDrivetrain.trackWidth > 0.0f) ? (vRight - vLeft) / mDrivetrain.trackWidth : 0.0f;

    std::lock_guard<pros::Mutex> lock(mDryRunMutex);
    float midHeading = mDryRunPose.theta + 0.5f * omega * dt;
    mDryRunPose.x += v * std::cos(midHeading) * dt;
    mDryRunPose.y += v * std::sin(midHeading) * dt;
    mDryRunPose.theta = wrapRad(mDryRunPose.theta + omega * dt);
    mDryRunPath.push_back(mDryRunPose);
}

void Chassis::setDryRun(bool enabled) {
    if (enabled) {
        Pose start = mOdom.getPose();
        {
            std::lock_guard<pros::Mutex> lock(mDryRunMutex);
            // Start from the real pose; the previewed auton's own setPose()
            // (if any) re-baselines it.
            mDryRunPose = start;
            mDryRunPath.clear();
            mDryRunPath.push_back(start);
        }
        mDryRunTask = pros::c::task_get_current();
    } else {
        mDryRunTask = nullptr;
    }
}

bool Chassis::isDryRun() const {
    return previewing();
}

const std::vector<Pose>& Chassis::dryRunPath() const {
    return mDryRunPath;
}

void Chassis::resetDryRunPath() {
    std::lock_guard<pros::Mutex> lock(mDryRunMutex);
    mDryRunPath.clear();
}

}  // namespace FBLIB
