#pragma once

#include <vector>

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

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

    // Distance traveled in inches since last reset
    float distanceIn() const;

    // Offset from robot tracking center (inches)
    // Sign convention: +right/+forward, -left/-backward
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

class OdomTracking {
public:
    OdomTracking() = default;
    OdomTracking(const OdomSensors& sensors);

    // Initialize / reconfigure sensors
    void setSensors(const OdomSensors& sensors);

    // Run one odometry update. Call at a fixed frequency (e.g. 10ms / 100Hz).
    // Reads all sensors, computes delta, and integrates into the current pose.
    void update();

    // Retrieve the decomposed delta from the last update() call.
    // dVert and dHoriz are independent sensor measurements (not geometrically
    // decomposed from a fused X/Y pose), allowing per-axis noise in MCL.
    OdomDelta getLastDelta() const { return mLastDelta; }

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

    // Direct pose access
    void setPose(const Pose& pose);
    Pose getPose() const;

    // Reset all tracking wheels and zero the pose
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

    // Previous readings for delta computation
    float mPrevVertDist{0.0f};     // accumulated vertical distance last update
    float mPrevHorizDist{0.0f};    // accumulated horizontal distance last update
    float mPrevHeadingRad{0.0f};   // IMU heading last update

    // Most recent decomposed delta, stored for MCL per-axis noise.
    // Updated at the end of each update() call.
    OdomDelta mLastDelta;

    // Helper: average distance across a wheel collection
    static float averageDistance(const std::vector<TrackingWheel*>& wheels);
    static float averageOffset(const std::vector<TrackingWheel*>& wheels);

    // Current vertical distance: tracking wheels → motor encoders → 0
    float currentVertDistance() const;

    // Current horizontal distance: tracking wheels → 0 (tank drives can't strafe)
    float currentHorizDistance() const;

    // Current heading from primary IMU (radians, standard math convention)
    float currentHeadingRad() const;
};

}  // namespace FBLIB
