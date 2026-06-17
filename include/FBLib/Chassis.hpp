#pragma once

#include <array>
#include <atomic>
#include <queue>
#include <vector>

#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include "FBLib/Movement_Control/MoveToPoint.hpp"
#include "FBLib/Movement_Control/TurnToPoint.hpp"
#include "FBLib/Movement_Control/Arc.hpp"
#include "FBLib/Movement_Control/Boomerang.hpp"
#include "FBLib/Movement_Control/RAMSETE.hpp"
#include "FBLib/Movement_Control/Velocity_Profiles.hpp"
#include "FBLib/Tracking/MCL_Tracking.hpp"
#include "FBLib/Tracking/Odom_Tracking.hpp"
#include "FBLib/Tracking/RCL_Tracking.hpp"
#include "FBLib/Util/Pid.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Drivetrain — hardware abstraction for the drive base
// ============================================================================

class Drivetrain {
public:
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float trackWidth, float wheelDiameter, float rpm = 450.0f,
               float horizontalDrift = 2.0f);

    pros::MotorGroup* leftMotors{nullptr};
    pros::MotorGroup* rightMotors{nullptr};
    float trackWidth;         // inches, distance between left and right wheels
    float wheelDiameter;      // inches
    float rpm;                // motor cartridge RPM
    float horizontalDrift;    // LemLib drift factor

    // Set motor voltages (-127 to 127)
    void setLeftVoltage(float voltage);
    void setRightVoltage(float voltage);
    void setVoltage(float left, float right);

    // Brake modes
    void setBrakeMode(pros::motor_brake_mode_e mode);

    // Get average motor positions (degrees)
    float averagePositionDeg() const;
};

// ============================================================================
// DriveCurve — joystick input curve for driver control
// ============================================================================

struct DriveCurve {
    float deadband{15.0f};      // ignore inputs below this
    float minOutput{20.0f};     // minimum output when above deadband
    float curve{1.0f};          // 1.0 = linear, >1.0 = exponential, <1.0 = logarithmic

    float apply(float input) const;
};

// ============================================================================
// ChassisConfig — all configuration needed to construct a Chassis
// ============================================================================

struct ChassisConfig {
    // PID gains
    PIDGains lateralGains{7.0f, 0.0f, 30.0f};
    PIDGains angularGains{4.0f, 0.0f, 37.7f};

    // Lateral PID settings
    float lateralWindupRange{0.0f};
    bool lateralFlipReset{false};

    // Angular PID settings
    float angularWindupRange{0.0f};
    bool angularFlipReset{false};

    // Drive curves (driver control)
    DriveCurve throttleCurve;
    DriveCurve steerCurve;

    // Default movement parameters
    MoveDistanceParams defaultMoveDistance;
    MoveToPointParams defaultMoveToPoint;
    TurntoHeadingParams defaultTurnToHeading;
    TurnToPointParams defaultTurnToPoint;

    // Distance sensors for MCL/RCL tracking
    // Size controlled by MAX_DISTANCE_SENSORS in Util.hpp
    std::array<pros::Distance*, MAX_DISTANCE_SENSORS> distanceSensors{};

    // Sensor mount offsets (robot-frame: x=forward, y=left, theta=pointing angle)
    // Index corresponds to distanceSensors index. Leave zero for unused sensors.
    std::array<Pose, MAX_DISTANCE_SENSORS> sensorMounts{};

    // Tracking
    bool useMclTracking{false};
    bool useRclTracking{false};
    MclTracking::Config mclConfig;
    RclTracking::Config rclConfig;
};

// ============================================================================
// Chassis — top-level robot abstraction
// ============================================================================
//
// Composes tracking, motion control, and driver control into a single API.
// This is the primary class users instantiate and interact with.
//
// Example:
//   Drivetrain dt(&leftMotors, &rightMotors, 10.4f, 3.25f);
//   OdomSensors sensors = { ... };
//   Chassis chassis(dt, sensors, config);
//   chassis.calibrate();
//   chassis.setPose({0, 0, 0});
//   chassis.moveToPoint(24, 0, 2000);  // blocking, 2s timeout
//   chassis.moveToPoint(48, 24, 0, {}, true);  // async, no timeout
// ============================================================================

class Chassis {
public:
    // ========================================================================
    // Construction
    // ========================================================================

    Chassis(Drivetrain& drivetrain, const OdomSensors& sensors,
            const ChassisConfig& config = {});

    ~Chassis();

    // ========================================================================
    // Calibration & Pose
    // ========================================================================

    /// Calibrate IMU (robot must be stationary for ~2 seconds)
    void calibrate();

    /// Set the robot's field pose
    void setPose(const Pose& pose);

    /// Set position only (heading unchanged)
    void setPosition(float x, float y);

    /// Set heading only (position unchanged)
    /// @param thetaDeg  heading in VEX degrees (0–360, clockwise positive)
    void setHeading(float thetaDeg);

    /// Get the current estimated pose
    Pose getPose() const;

    // ========================================================================
    // Driver Control
    // ========================================================================

    /// Tank drive: left and right joystick values (-127 to 127)
    void tank(float left, float right);

    /// Arcade drive: throttle and steer (-127 to 127)
    void arcade(float throttle, float steer);

    /// Curvature drive: like arcade but with better turn handling
    void curvature(float throttle, float steer);

    // ========================================================================
    // Autonomous Movement (blocking by default, async overload available)
    // ========================================================================
    //
    // All heading values are in VEX degrees (0–360, clockwise positive).
    // timeout in milliseconds — 0 = no timeout.
    // ========================================================================

    /// Drive a specific distance forward/backward
    void moveDistance(float target, float timeout = 0, const MoveDistanceParams& params = {}, bool async = false);

    /// Drive to a field coordinate
    void moveToPoint(float x, float y, float timeout = 0, const MoveToPointParams& params = {}, bool async = false);

    /// Turn to an absolute heading (VEX degrees, 0–360 clockwise)
    void turnToHeading(float thetaDeg, float timeout = 0, const TurntoHeadingParams& params = {}, bool async = false);

    /// Turn to face a field coordinate
    void turnToPoint(float x, float y, float timeout = 0, const TurnToPointParams& params = {}, bool async = false);

    /// Drive an arc to a target point with a given radius
    void moveArc(float x, float y, float radius, float timeout = 0, const ArcParams& params = {}, bool async = false);

    /// Boomerang: curved approach to a pose
    /// @param thetaDeg  final heading in VEX degrees (0–360, clockwise positive)
    void moveBoomerang(float x, float y, float thetaDeg, float timeout = 0, const BoomerangParams& params = {}, bool async = false);

    /// Follow a path of poses using RAMSETE
    void moveRAMSETE(const std::vector<Pose>& path, float timeout = 0, const RAMSETEParams& params = {}, bool async = false);

    /// Swing turn using one side of the drive
    /// @param thetaDeg  target heading in VEX degrees (0–360, clockwise positive)
    void swingToHeading(float thetaDeg, SwingSide side, float timeout = 0, const SwingParams& params = {}, bool async = false);

    // ========================================================================
    // Motion Status
    // ========================================================================

    /// True if the current movement has completed
    bool isSettled() const;

    /// Cancel the current async movement
    void cancelMotion();

    /// Block until the current movement completes
    void waitUntilSettled();

    /// Wait until the robot has traveled at least `dist` inches
    void waitUntilDist(float dist);

    // ========================================================================
    // Tracking Access (advanced use)
    // ========================================================================

    OdomTracking& odom();
    RclTracking& rcl();
    MclTracking& mcl();

    /// Start/stop all tracking systems
    void startTracking();
    void stopTracking();

    // ========================================================================
    // Tuning
    // ========================================================================

    void setLateralGains(const PIDGains& gains);
    void setAngularGains(const PIDGains& gains);
    void setThrottleCurve(const DriveCurve& curve);
    void setSteerCurve(const DriveCurve& curve);

    /// Set brake mode for all drive motors
    void setBrakeMode(pros::motor_brake_mode_e mode);

    /// Set/replace distance sensors for MCL/RCL tracking
    void setDistanceSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors);

    // ========================================================================
    // Dry-run mode (for Auton Selector path preview)
    // ========================================================================

    /// Enable dry-run mode: movement commands record their paths but don't move motors
    void setDryRun(bool enabled);
    bool isDryRun() const;

    /// Get the recorded path from the last dry run
    const std::vector<Pose>& dryRunPath() const;

    /// Reset the dry-run path recording
    void resetDryRunPath();

private:
    // ========================================================================
    // Internal motion helpers
    // ========================================================================

    void startMotion();
    void endMotion();

    float computeLateralError(float targetX, float targetY) const;
    float computeAngularError(float targetHeadingRad) const;
    float computeAngularErrorToPoint(float targetX, float targetY) const;
    // ========================================================================
    // Motion task (for async operation)
    // ========================================================================

    void runMotionTask();
    void executeCurrentMotion();
    void ensureMotionTask();
    enum class MotionType { None, Distance, Point, Heading, TurnPoint, Arc, Boomerang, RAMSETE, Swing };
    struct MotionRequest {
        MotionType type{MotionType::None};
        float targetX{0}, targetY{0}, targetHeadingRad{0}, targetRadius{0};
        float targetDistance{0};         // raw distance for Distance motion (computed at exec time)
        float timeout{0};                // milliseconds, 0 = no timeout
        uint32_t startTime{0};           // pros::millis() when motion began
        MoveDistanceParams moveDistParams;
        MoveToPointParams movePointParams;
        TurntoHeadingParams turnHeadingParams;
        TurnToPointParams turnPointParams;
        ArcParams arcParams;
        BoomerangParams boomerangParams;
        RAMSETEParams ramseteParams;
        SwingParams swingParams;
        SwingSide swingSide{SwingSide::Left};
        std::vector<Pose> ramsetePath;
    };

    // ========================================================================
    // Members
    // ========================================================================

    Drivetrain& mDrivetrain;
    ChassisConfig mConfig;

    // Tracking
    OdomTracking mOdom;
    RclTracking mRcl;
    MclTracking mMcl;

    // PID controllers
    PID mLateralPID;
    PID mAngularPID;

    // Motion state
    std::atomic<bool> mMotionRunning{false};
    std::atomic<bool> mMotionCancelled{false};
    std::atomic<bool> mTaskShouldStop{false};
    MotionRequest mCurrentMotion;
    std::queue<MotionRequest> mMotionQueue;
    pros::Mutex mQueueMutex;
    pros::Task* mMotionTask{nullptr};

    // Drive curves
    DriveCurve mThrottleCurve;
    DriveCurve mSteerCurve;

    // Dry-run mode
    bool mDryRun{false};
    std::vector<Pose> mDryRunPath;
    Pose mDryRunStartPose;
    Pose mDryRunPose;  // synthetic pose for dry-run simulation
};

}  // namespace FBLIB
