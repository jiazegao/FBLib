#pragma once

#include <cmath>

#include "pros/imu.hpp"

namespace FBLIB {

// ============================================================================
// ScaledIMU — V5 Inertial Sensor with calibration scale correction
// ============================================================================
//
// VEX V5 IMUs typically under-report rotation (e.g. reading 354.25° when the
// robot physically turns 360°).  This class applies a multiplicative scale
// factor to get_rotation() and get_heading() so that a commanded turn produces
// the correct angular displacement.
//
// Usage:
//   ScaledIMU imu(15, 360.0, 354.25);  // port 15, 360° expected, 354.25° actual
//
// To find your IMU's actual_reading:
//   1. Place the robot on a turntable or known-angle surface.
//   2. Rotate the robot exactly 10 full turns (3600°).
//   3. Read imu.get_rotation() / 10.0 to get the per-revolution reading.
//   4. Pass that value as actual_reading.
//
// IMPORTANT: pros::Imu methods are NOT virtual.  If a ScaledIMU* is passed
// through a pros::Imu* pointer (e.g. in OdomSensors::imuCollection), the
// base-class methods will be called — NOT the scaled overrides.  To use
// scaling through the odometry pipeline, set OdomSensors::imuScaleFactor
// instead (or in addition to) using this class.
// ============================================================================

class ScaledIMU : public pros::Imu {
public:
    /// Constructor.
    /// @param port             V5 Smart Port number (1–21)
    /// @param expectedRotation The commanded rotation (default 360.0°)
    /// @param actualReading    What the IMU actually reports for that rotation
    ///                         (e.g. 354.25°).  Must be > 0.
    ScaledIMU(std::uint8_t port,
              double expectedRotation = 360.0,
              double actualReading = 355.0)
        : pros::Imu(port),
          mScaleFactor(expectedRotation / actualReading)
    {}

    /// Return the scale factor (expected / actual).
    double scaleFactor() const { return mScaleFactor; }

    /// Update the scale factor at runtime (e.g. after re-calibration).
    void setScaleFactor(double expectedRotation, double actualReading) {
        if (actualReading > 0.0) {
            mScaleFactor = expectedRotation / actualReading;
        }
    }

    /// Return the scaled continuous rotation (degrees).
    /// Positive = clockwise (VEX convention).
    double get_rotation() const {
        double raw = pros::Imu::get_rotation();
        if (std::isinf(raw)) return raw;  // PROS error sentinel
        return raw * mScaleFactor;
    }

    /// Return the scaled bounded heading [0, 360) degrees.
    double get_heading() const {
        double scaled = get_rotation();
        if (std::isinf(scaled)) return scaled;

        double heading = std::fmod(scaled, 360.0);
        if (heading < 0.0) heading += 360.0;
        return heading;
    }

private:
    double mScaleFactor;
};

}  // namespace FBLIB
