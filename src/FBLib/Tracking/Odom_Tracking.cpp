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
    // update() computes a near-zero delta.  Without this, the first update
    // sees a jump equal to the entire accumulated tracking-wheel distance
    // plus a 90° heading offset (IMU 0° VEX = PI/2 standard radians).
    mPrevVertDist   = currentVertDistance();
    mPrevHorizDist  = currentHorizDistance();
    mPrevHeadingRad = currentHeadingRad();
}

void OdomTracking::setSensors(const OdomSensors& sensors) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors = sensors;
    mPrevVertDist  = currentVertDistance();
    mPrevHorizDist = currentHorizDistance();
    mPrevHeadingRad = currentHeadingRad();
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

float OdomTracking::currentHeadingRad() const {
    if (!mSensors.imuCollection.empty() && mSensors.imuCollection[0] != nullptr) {
        float rawHeading = mSensors.imuCollection[0]->get_heading();
        // PROS returns PROS_ERR_F (== INFINITY) on an IMU fault (disconnect,
        // transient error, mid-calibration). Feeding that downstream corrupts
        // the pose; hold the last good heading instead of integrating garbage.
        if (!std::isfinite(rawHeading)) return mPrevHeadingRad;
        // Apply calibration scale factor to correct IMU under-reporting.
        // V5 IMUs typically read ~354.25° for a 360° physical turn.
        float scaledHeading = rawHeading * mSensors.imuScaleFactor;
        return vexToStdRad(scaledHeading);
    }
    return 0.0f;
}

// ============================================================================
// update() — one odometry tick
// ============================================================================

void OdomTracking::update() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    float headingRad = currentHeadingRad();
    float vertDist   = currentVertDistance();
    float horizDist  = currentHorizDistance();

    // — Compute deltas since last update —
    float dThetaRad = headingRad - mPrevHeadingRad;
    dThetaRad = wrapRad(dThetaRad);

    float dVert  = vertDist  - mPrevVertDist;
    float dHoriz = horizDist - mPrevHorizDist;

    // — Arc approximation for field-frame displacement —
    float midHeadingRad = mPrevHeadingRad + dThetaRad * 0.5f;

    float dX = 0.0f;
    float dY = 0.0f;

    if (std::fabs(dThetaRad) < 1e-6f) {
        // Negligible rotation → use simple trig (no arc)
        dX = dVert * std::cos(mPrevHeadingRad) - dHoriz * std::sin(mPrevHeadingRad);
        dY = dVert * std::sin(mPrevHeadingRad) + dHoriz * std::cos(mPrevHeadingRad);
    } else {
        // Arc approximation with midpoint heading
        dX = dVert * std::cos(midHeadingRad) - dHoriz * std::sin(midHeadingRad);
        dY = dVert * std::sin(midHeadingRad) + dHoriz * std::cos(midHeadingRad);
    }

    // — Integrate into pose —
    mPose.x += dX;
    mPose.y += dY;
    mPose.theta = headingRad;

    // — Store for next update —
    mPrevVertDist   = vertDist;
    mPrevHorizDist  = horizDist;
    mPrevHeadingRad = headingRad;

    // — Expose decomposed delta for MCL per-axis noise —
    mLastDelta = {dVert, dHoriz, dThetaRad};

    // — Accumulate for consumeDelta() so consumers polling slower than the
    //   update rate (e.g. MCL at 25ms vs odometry at 10ms) never miss motion —
    mAccumDelta.dVert  += dVert;
    mAccumDelta.dHoriz += dHoriz;
    mAccumDelta.dTheta += dThetaRad;
}

OdomDelta OdomTracking::consumeDelta() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    OdomDelta out = mAccumDelta;
    mAccumDelta = {};
    return out;
}

void OdomTracking::setPose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose = pose;
    // Re-sync accumulated distances to current sensor readings so the next
    // delta starts from zero (tracking wheels → motor encoders → 0).
    mPrevVertDist  = currentVertDistance();
    mPrevHorizDist = currentHorizDistance();
    // IMPORTANT: do NOT set mPrevHeadingRad here.  mPrevHeadingRad tracks
    // the raw IMU heading, not the odometry pose heading.  Overwriting it
    // with pose.theta would cause the next update() to compute a spurious
    // dThetaRad equal to imuHeading - pose.theta (the sync correction of any
    // tracking system that called setPose, e.g. MCL syncToOdometry).
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
    mPose = Pose{};
    // Re-baseline: for motor encoders (which can't be reset), store the
    // current reading so the next delta starts from zero.
    mPrevVertDist   = currentVertDistance();
    mPrevHorizDist  = currentHorizDistance();
    // Read current IMU heading so the next update() computes a near-zero
    // delta.  Setting this to 0.0f would cause a ~90° spurious rotation
    // on the first update after reset (IMU 0° VEX = PI/2 rad).
    mPrevHeadingRad = currentHeadingRad();
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
    if (!mSensors.imuCollection.empty() && mSensors.imuCollection[0] != nullptr) {
        float rawHeading = mSensors.imuCollection[0]->get_heading();
        return rawHeading * mSensors.imuScaleFactor;
    }
    return 0.0f;
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
