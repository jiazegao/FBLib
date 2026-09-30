#include "FBLib/Tracking/Odom_Tracking.hpp"

#include <cmath>
#include <mutex>

#include "FBLib/Chassis.hpp"
#include "pros/rtos.hpp"

namespace FBLIB {

// ============================================================================
// TrackingWheel
// ============================================================================

// ADI encoder: 360 ticks per revolution
static constexpr float ADI_TICKS_PER_REV = 360.0f;

// Rotation sensor: reports centidegrees (100 per degree, 36000 per revolution)
static constexpr float ROTATION_UNITS_PER_REV = 36000.0f;

TrackingWheel::TrackingWheel(pros::adi::Encoder& enc,
                             float wheelDiamIn,
                             float offsetIn,
                             float gearRatio)
    : mType(SensorType::ADI),
      mAdi(&enc),
      mWheelDiamIn(wheelDiamIn),
      mOffsetIn(offsetIn),
      mGearRatio(gearRatio) {}

TrackingWheel::TrackingWheel(pros::Rotation& rot,
                             float wheelDiamIn,
                             float offsetIn,
                             float gearRatio)
    : mType(SensorType::Rotation),
      mRot(&rot),
      mWheelDiamIn(wheelDiamIn),
      mOffsetIn(offsetIn),
      mGearRatio(gearRatio) {}

float TrackingWheel::distanceIn() const {
    // Circumference of the tracking wheel
    float circumference = mWheelDiamIn * PI;

    // PROS reports INT32_MAX (PROS_ERR) when the sensor errors or is
    // unplugged. Integrating that would teleport odometry ~150,000 miles;
    // hold the last good distance instead.
    constexpr int32_t kProsErr = INT32_MAX;

    if (mType == SensorType::ADI && mAdi != nullptr) {
        // ADI encoder: ticks → revolutions → distance
        int32_t rawTicks = mAdi->get_value();
        if (rawTicks == kProsErr) return mLastGoodDistIn;
        float revolutions = static_cast<float>(rawTicks) / (ADI_TICKS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        return mLastGoodDistIn;
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        // Rotation sensor: centidegrees → revolutions → distance
        int32_t rawCentideg = mRot->get_position();
        if (rawCentideg == kProsErr) return mLastGoodDistIn;
        float revolutions = static_cast<float>(rawCentideg) / (ROTATION_UNITS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        return mLastGoodDistIn;
    }
    return 0.0f;
}

float TrackingWheel::rawTicks() const {
    if (mType == SensorType::ADI && mAdi != nullptr) {
        return static_cast<float>(mAdi->get_value());
    }
    return 0.0f;
}

float TrackingWheel::rawDegrees() const {
    if (mType == SensorType::Rotation && mRot != nullptr) {
        return static_cast<float>(mRot->get_position()) * 0.01f;  // centidegrees → degrees
    }
    return 0.0f;
}

void TrackingWheel::reset() {
    if (mType == SensorType::ADI && mAdi != nullptr) {
        mAdi->reset();
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        mRot->reset_position();
    }
}

// ============================================================================
// OdomTracking
// ============================================================================

OdomTracking::OdomTracking(const OdomSensors& sensors)
    : mSensors(sensors), mPose() {
    // Initialize previous readings from current sensor values so the first
    // update() computes a near-zero delta, and start facing the IMU heading.
    mPrevVertDist    = currentVertDistance();
    mPrevHorizDist   = currentHorizDistance();
    mPrevRotationDeg = currentRotationDeg();
    mPose.theta      = vexToStdRad(mPrevRotationDeg);
}

void OdomTracking::setSensors(const OdomSensors& sensors) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors = sensors;
    mPrevVertDist    = currentVertDistance();
    mPrevHorizDist   = currentHorizDistance();
    mPrevRotationDeg = currentRotationDeg();
}

// ============================================================================
// Sensor read helpers — each has a fallback chain
// ============================================================================

float OdomTracking::currentVertDistance() const {
    // 1. External tracking wheels
    if (!mSensors.vertWheelCollection.empty()) {
        return averageDistance(mSensors.vertWheelCollection);
    }
    // 2. Integrated motor encoders (drivetrain fallback)
    if (mSensors.drivetrain != nullptr) {
        float avgDeg = mSensors.drivetrain->averagePositionDeg();
        float motorRevolutions = avgDeg / 360.0f;
        // Motor encoders measure the MOTOR shaft. With an external gear ratio
        // (e.g. 600 RPM blue cartridge driving a 450 RPM wheel), one motor rev
        // is only wheelRPM/cartridgeRPM wheel revs. Omitting this factor
        // overestimated distance by 33% on a 600→450 geared drive.
        float wheelRevolutions = motorRevolutions *
            mSensors.drivetrain->externalGearRatio();
        return wheelRevolutions * mSensors.drivetrain->wheelDiameter * PI;
    }
    // 3. No vertical tracking source
    return 0.0f;
}

float OdomTracking::currentHorizDistance() const {
    // 1. External tracking wheels
    if (!mSensors.horizWheelCollection.empty()) {
        return averageDistance(mSensors.horizWheelCollection);
    }
    // 2. Tank drives cannot measure lateral movement from motor encoders
    return 0.0f;
}

float OdomTracking::currentRotationDeg() const {
    if (!mSensors.imuCollection.empty() && mSensors.imuCollection[0] != nullptr) {
        // Continuous rotation, NOT the wrapped 0-360 heading: scaling a
        // wrapped value makes it jump by 360*(scale-1) degrees every time the
        // robot crosses north.
        double rotation = mSensors.imuCollection[0]->get_rotation();
        // PROS returns PROS_ERR_F (== INFINITY) on an IMU fault (disconnect,
        // transient error, mid-calibration). Hold the last good reading
        // instead of integrating garbage.
        if (!std::isfinite(rotation)) return mPrevRotationDeg;
        // Apply calibration scale factor to correct IMU under-reporting.
        // V5 IMUs typically read ~354.25° for a 360° physical turn.
        return static_cast<float>(rotation) * mSensors.imuScaleFactor;
    }
    return 0.0f;
}

float OdomTracking::vertOffset() const {
    // Motor encoders average both sides, which cancels rotation already.
    return mSensors.vertWheelCollection.empty() ? 0.0f : averageOffset(mSensors.vertWheelCollection);
}

float OdomTracking::horizOffset() const {
    return averageOffset(mSensors.horizWheelCollection);
}

// ============================================================================
// update() — one odometry tick
// ============================================================================

void OdomTracking::update() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    float rotationDeg = currentRotationDeg();
    float vertDist    = currentVertDistance();
    float horizDist   = currentHorizDistance();

    // — Compute deltas since last update —
    // VEX rotation is clockwise-positive; pose heading is counter-clockwise.
    float dThetaRad = wrapRad(-degToRad(rotationDeg - mPrevRotationDeg));

    // A wheel mounted away from the tracking center also rolls when the
    // robot only rotates: a vertical wheel `o` inches right of center moves
    // forward o*dTheta in a CCW turn; a horizontal wheel `o` inches ahead of
    // center moves left o*dTheta. Remove that so only the CENTER's motion is
    // integrated (otherwise every turn adds phantom translation).
    float dVert  = (vertDist  - mPrevVertDist)  - vertOffset()  * dThetaRad;
    float dHoriz = (horizDist - mPrevHorizDist) - horizOffset() * dThetaRad;

    // — Arc integration in the FIELD frame —
    // Motion along an arc of angle dTheta covers a chord 2*sin(dTheta/2)/dTheta
    // of its length, in the direction of the average (mid-tick) heading.
    float chordScale = (std::fabs(dThetaRad) < 1e-6f)
        ? 1.0f
        : 2.0f * std::sin(0.5f * dThetaRad) / dThetaRad;
    float midHeadingRad = mPose.theta + 0.5f * dThetaRad;
    float cosMid = std::cos(midHeadingRad);
    float sinMid = std::sin(midHeadingRad);

    // Robot frame: vertical = forward, horizontal = left
    mPose.x += chordScale * (dVert * cosMid - dHoriz * sinMid);
    mPose.y += chordScale * (dVert * sinMid + dHoriz * cosMid);
    mPose.theta = wrapRad(mPose.theta + dThetaRad);

    // — Store for next update —
    mPrevVertDist    = vertDist;
    mPrevHorizDist   = horizDist;
    mPrevRotationDeg = rotationDeg;

    // — Expose decomposed delta for MCL per-axis noise —
    mLastDelta = {dVert, dHoriz, dThetaRad};

    // — Accumulate for consumeDelta() so consumers polling slower than the
    //   update rate (e.g. MCL at 25ms vs odometry at 10ms) never miss motion —
    mAccumDelta.dVert  += dVert;
    mAccumDelta.dHoriz += dHoriz;
    mAccumDelta.dTheta += dThetaRad;
}

OdomDelta OdomTracking::getLastDelta() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mLastDelta;
}

OdomDelta OdomTracking::consumeDelta() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    OdomDelta out = mAccumDelta;
    mAccumDelta = {};
    return out;
}

// Pose setters do NOT re-baseline the sensors: the next update() applies the
// motion since the previous tick on top of the new pose. Re-baselining here
// silently dropped that motion — and MCL/RCL sync many times a second.

void OdomTracking::setPose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose = {pose.x, pose.y, wrapRad(pose.theta)};
}

void OdomTracking::setPosition(float x, float y) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose.x = x;
    mPose.y = y;
}

void OdomTracking::setHeading(float thetaRad) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose.theta = wrapRad(thetaRad);
}

void OdomTracking::translate(float dx, float dy) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose.x += dx;
    mPose.y += dy;
}

Pose OdomTracking::getPose() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mPose;
}

void OdomTracking::reset() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    // Reset all tracking wheels (if present)
    for (auto* wheel : mSensors.vertWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    for (auto* wheel : mSensors.horizWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    // Re-baseline: for motor encoders (which can't be reset), store the
    // current reading so the next delta starts from zero.
    mPrevVertDist    = currentVertDistance();
    mPrevHorizDist   = currentHorizDistance();
    mPrevRotationDeg = currentRotationDeg();
    // Zero the position and face the way the IMU says (IMU 0° = +Y). Set it
    // here rather than waiting for the next update() so a motion started
    // straight after calibrate() doesn't see a stale heading.
    mPose = {0.0f, 0.0f, vexToStdRad(mPrevRotationDeg)};
    // Drop any motion accumulated before the reset — it belongs to the old
    // baseline and would otherwise be applied to consumers after re-zeroing.
    mLastDelta = {};
    mAccumDelta = {};
}

void OdomTracking::calibrate() {
    // Calibrate all IMUs.
    // Use blocking mode (true) so we don't need a blind delay — the call returns
    // only after calibration completes (~2s).  Without blocking, reset(false)
    // starts calibration but returns immediately, leading to premature reads.
    for (auto* imu : mSensors.imuCollection) {
        if (imu != nullptr) {
            imu->reset(true);
        }
    }
    reset();
}

float OdomTracking::imuHeadingDeg() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return wrapDeg(currentRotationDeg());
}

float OdomTracking::imuHeadingRad() const {
    return vexToStdRad(imuHeadingDeg());
}

float OdomTracking::averageDistance(const std::vector<TrackingWheel*>& wheels) {
    if (wheels.empty()) return 0.0f;

    float sum = 0.0f;
    int count = 0;
    for (const auto* wheel : wheels) {
        if (wheel != nullptr) {
            sum += wheel->distanceIn();
            ++count;
        }
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
}

float OdomTracking::averageOffset(const std::vector<TrackingWheel*>& wheels) {
    if (wheels.empty()) return 0.0f;

    float sum = 0.0f;
    int count = 0;
    for (const auto* wheel : wheels) {
        if (wheel != nullptr) {
            sum += wheel->offsetIn();
            ++count;
        }
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
}

}  // namespace FBLIB
