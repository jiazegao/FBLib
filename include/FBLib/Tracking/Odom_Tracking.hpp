#pragma once

#include <vector>

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// Forward declaration — full definition in Chassis.hpp
class Drivetrain;

// ============================================================================
// TrackingWheel — wraps an ADI encoder or V5 Rotation sensor as a dead wheel
// ============================================================================

class TrackingWheel {
public:
    enum class SensorType { ADI, Rotation };

    // ADI encoder constructor
    TrackingWheel(pros::adi::Encoder& enc,
                  float wheelDiamIn,
                  float offsetIn,
                  float gearRatio = 1.0f);

    // Rotation sensor constructor
    TrackingWheel(pros::Rotation& rot,
                  float wheelDiamIn,
                  float offsetIn,
                  float gearRatio = 1.0f);

    // Distance traveled in inches since last reset.
    // Vertical wheels must read positive driving forward; horizontal wheels
    // positive moving LEFT (reverse the sensor if needed).
    float distanceIn() const;

    // Offset from robot tracking center (inches)
    // Sign convention: +right/+forward, -left/-backward
    //   vertical wheel:   lateral offset,      + = right of center
    //   horizontal wheel: longitudinal offset, + = ahead of center
    float offsetIn() const { return mOffsetIn; }

    // Wheel diameter in inches
    float wheelDiamIn() const { return mWheelDiamIn; }

    // Gear ratio between wheel and sensor (e.g. 1.0 = direct, 3.0/5.0 = geared up)
    float gearRatio() const { return mGearRatio; }

    // Sensor type
    SensorType sensorType() const { return mType; }

    // Reset accumulated distance to zero
    void reset();

    // Raw tick/angle access for advanced use
    float rawTicks() const;
    float rawDegrees() const;

private:
    SensorType mType;
    pros::adi::Encoder* mAdi{nullptr};
    pros::Rotation* mRot{nullptr};

    float mWheelDiamIn{0.0f};
    float mOffsetIn{0.0f};
    float mGearRatio{1.0f};

    // Last valid distance — returned when the sensor reports PROS_ERR
    // (INT32_MAX) so a transient disconnect holds position instead of
    // teleporting odometry by ~2 billion ticks. Mutable: distanceIn() is
    // logically const.
    mutable float mLastGoodDistIn{0.0f};
};

// ============================================================================
// OdomSensors — collection of all sensors used for odometry
// ============================================================================

struct OdomSensors {
    std::vector<TrackingWheel*> vertWheelCollection;   // forward/backward tracking wheels
    std::vector<TrackingWheel*> horizWheelCollection;  // sideways tracking wheels
    std::vector<pros::Imu*> imuCollection;             // IMUs (first one is primary)

    // Fallback: if no tracking wheels are present, motor encoders are used
    // for vertical (forward/backward) distance. Set to nullptr if unused.
    Drivetrain* drivetrain{nullptr};

    // IMU calibration scale factor.  V5 IMUs typically under-report rotation
    // (e.g. 354.25° when physically turning 360°).  Set this to
    //   expectedRotation / actualReading   (e.g. 360.0 / 354.25 = 1.0162)
    // to correct all heading readings through the odometry pipeline.
    // Default 1.0 = no scaling.  Also see ScaledIMU for direct IMU use.
    float imuScaleFactor{1.0f};
};

// ============================================================================
// OdomTracking — dead-wheel + IMU odometry solver
// ============================================================================
//
// The IMU is authoritative for ROTATION: every update adds the IMU's change
// in heading to the pose heading. The pose heading itself is whatever the
// user (or calibrate()) set, so setPose()/setHeading() define which way the
// robot faces on the field.
// ============================================================================

class OdomTracking {
public:
    OdomTracking() = default;
    OdomTracking(const OdomSensors& sensors);

    // Initialize / reconfigure sensors
    void setSensors(const OdomSensors& sensors);

    // Run one odometry update. Call at a fixed frequency (e.g. 10ms / 100Hz).
    // Reads all sensors, computes delta, and integrates into the current pose.
    // Thread-safe: internally serialized — but note that every update() call
    // consumes the sensor-delta baseline, so consumers needing motion deltas
    // must use consumeDelta() (accumulated), never getLastDelta(), when more
    // than one task may be calling update().
    void update();

    // Retrieve the decomposed delta from the last update() call: motion of the
    // tracking center in the robot frame (wheel offsets already removed).
    // WARNING: single-tick only. If update() runs more than once between two
    // reads, the intermediate deltas are LOST. Use consumeDelta() instead.
    OdomDelta getLastDelta() const;

    // Retrieve the ACCUMULATED decomposed delta since the previous call to
    // consumeDelta(), and reset the accumulator. Thread-safe. This is what
    // MCL uses to propagate particles — it never misses motion regardless of
    // how many update() ticks ran between MCL iterations.
    OdomDelta consumeDelta();

    // Whether horizontal tracking is available (dedicated wheel, not zero-fallback).
    bool hasHorizontalTracking() const {
        return !mSensors.horizWheelCollection.empty();
    }

    // Whether vertical tracking uses dedicated wheels (as opposed to motor encoders).
    // When false, MCL should use IME variance instead of tracking wheel variance
    // for the vertical axis — motor encoders have more slip than dead wheels.
    bool hasVerticalTrackingWheel() const {
        return !mSensors.vertWheelCollection.empty();
    }

    // Direct pose access (all thread-safe)
    void setPose(const Pose& pose);
    void setPosition(float x, float y);       // heading unchanged
    void setHeading(float thetaRad);          // position unchanged
    void translate(float dx, float dy);       // atomic position correction (MCL/RCL sync)
    Pose getPose() const;

    // Reset all tracking wheels, zero the position and align the heading with
    // the IMU (IMU 0° = facing +Y)
    void reset();

    // Calibrate IMUs (call while robot is stationary)
    void calibrate();

    // Access underlying sensors
    const OdomSensors& sensors() const { return mSensors; }

    // Get the primary IMU heading (degrees, VEX convention: 0=forward, CW positive)
    float imuHeadingDeg() const;

    // Get heading in standard math radians
    float imuHeadingRad() const;

private:
    OdomSensors mSensors;
    Pose mPose;

    // Serializes update()/setPose()/getPose()/reset()/consumeDelta() across
    // tasks. Multiple tasks touch odometry concurrently (motion task, MCL,
    // RCL, user code) — unserialized read-modify-write of mPose/mPrev* would
    // corrupt the pose. Mutable so const accessors (getPose) can lock.
    mutable pros::Mutex mMutex;

    // Previous readings for delta computation
    float mPrevVertDist{0.0f};     // accumulated vertical distance last update
    float mPrevHorizDist{0.0f};    // accumulated horizontal distance last update
    float mPrevRotationDeg{0.0f};  // scaled IMU rotation last update (VEX, CW+)

    // Most recent decomposed delta, stored for MCL per-axis noise.
    // Updated at the end of each update() call.
    OdomDelta mLastDelta;

    // Accumulated delta since last consumeDelta() — survives multiple
    // update() ticks so consumers never miss motion.
    OdomDelta mAccumDelta;

    // Helper: average distance across a wheel collection
    static float averageDistance(const std::vector<TrackingWheel*>& wheels);
    static float averageOffset(const std::vector<TrackingWheel*>& wheels);

    // Current vertical distance: tracking wheels → motor encoders → 0
    float currentVertDistance() const;

    // Current horizontal distance: tracking wheels → 0 (tank drives can't strafe)
    float currentHorizDistance() const;

    // Continuous scaled IMU rotation (degrees, VEX clockwise-positive).
    // Holds the previous value while the IMU reports an error.
    float currentRotationDeg() const;

    // Offsets of the wheels actually used for each axis (0 for motor encoders)
    float vertOffset() const;
    float horizOffset() const;
};

}  // namespace FBLIB
