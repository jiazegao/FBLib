#pragma once

#include <array>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "FBLib/Tracking/Odom_Tracking.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// RclTracking — Ray/Ceiling-Line Localization using V5 distance sensors
// ============================================================================
//
// Uses distance sensors pointed at field walls and ceiling bar to determine
// absolute position by intersecting sensor rays with known field geometry.
// This provides a direct global position estimate that can be used to:
//   - Initialize MCL particles at match start
//   - Correct odometry drift during autonomous
//   - Detect if the robot has been pushed/grabbed
//
// Typical configuration: 8 distance sensors around the robot perimeter.
// ============================================================================

/// Which coordinate a single sensor constrains.
/// A sensor pointed at an east/west wall determines X; a sensor pointed at a
/// north/south wall determines Y.  A single distance sensor cannot determine
/// both coordinates — the unconstrained coordinate must come from another
/// sensor or from odometry.
enum class CoordType { X, Y, INVALID };

class RclTracking {
public:
    // ========================================================================
    // RclSensor — a single distance sensor with known mount pose
    // ========================================================================

    struct RclSensor {
        pros::Distance* sensor{nullptr};
        Pose mountOffset;          // robot-frame offset (x=forward, y=left, theta=pointing angle)
        bool enabled{true};

        // Computed world-space values (updated each cycle)
        float worldX{0.0f}, worldY{0.0f};
        float rayCos{0.0f}, raySin{0.0f};
        float worldHeading{0.0f};  // world-frame ray direction (radians)
        float readingInch{0.0f};
        int confidence{0};
        bool valid{false};
    };

    // ========================================================================
    // Obstacle types (can block a sensor's view of field walls)
    // ========================================================================

    struct LineObstacle {
        float x1, y1, x2, y2;
        float lifetimeMs = 0.0f;  // 0 = permanent, >0 = temporary
    };

    struct CircleObstacle {
        float x, y, radius;
        float lifetimeMs = 0.0f;
    };

    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config {
        int sensorCount = MAX_DISTANCE_SENSORS;
        float maxSyncDist = 5.0f;          // max distance (inches) to sync per update
        float accumulationAlpha = 0.3f;    // exponential moving average factor
        float updatePeriodMs = 10.0f;      // background task interval
        float angleTolerance = 10.0f;      // degrees — sensor ray must be within this
                                           //   of a wall axis (0°, 90°, 180°, 270°)
        int confidenceThreshold = 30;      // min V5 sensor confidence (0-63)
        bool autoSync = true;              // automatically sync pose to odometry
        bool useAccumulation = false;      // average multiple readings for stability
    };

    // ========================================================================
    // Constructor / Destructor
    // ========================================================================

    RclTracking(OdomTracking& odom, const Config& config);
    ~RclTracking();

    // ========================================================================
    // Sensor management
    // ========================================================================

    /// Add/replace a sensor at the given index
    void setSensor(int index, pros::Distance* sensor, const Pose& mountOffset);

    /// Configure all sensors from parallel arrays (distance sensors + mount offsets).
    /// Non-null distance entries are enabled with the corresponding mount offset;
    /// null entries are disabled. sensorCount is set to the count of
    /// non-null entries (minimum 1, maximum MAX_DISTANCE_SENSORS).
    void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                          const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts);

    /// Enable/disable a specific sensor
    void enableSensor(int index);
    void disableSensor(int index);
    void disableSensorFor(int index, float durationMs);

    // ========================================================================
    // Obstacle management
    // ========================================================================

    /// Add an obstacle that can block sensor lines-of-sight
    void addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs = 0.0f);
    void addCircleObstacle(float x, float y, float radius, float lifetimeMs = 0.0f);
    void clearObstacles();

    // ========================================================================
    // Core RCL operations
    // ========================================================================

    /// Run one localization update cycle (read sensors → compute position)
    void update();

    /// Update world-space positions of all sensors based on current robot pose
    void updateSensorPoses(const Pose& robotPose);

    /// Validate a sensor reading (range, confidence, obstacle intersection)
    bool isValidReading(int index) const;

    /// Compute robot coordinate from a single sensor intersecting a field wall.
    /// A sensor pointed at an east/west wall returns X; at a north/south wall
    /// returns Y.  A single distance sensor can only constrain one dimension.
    /// Returns {CoordType, coordinate} — CoordType::INVALID if no valid intersection.
    std::pair<CoordType, float> getBotCoordFromSensor(int index) const;

    /// Derive robot position from all valid sensors.
    /// X and Y are independently averaged from sensors that constrain each axis.
    /// When no sensor constrains an axis, the odometry value is used.
    /// Returns {pose, numSensorsUsed}
    std::pair<Pose, int> computePose() const;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Start background localization task
    void startTracking();

    /// Stop background task
    void stopTracking();

    /// Check if running
    bool isTracking() const;

    // ========================================================================
    // Sync
    // ========================================================================

    /// Smoothly blend RCL estimate into odometry pose
    void syncUpdate();

    /// Directly set pose from the best available sensor reading (for initialization)
    bool updateBotPoseFromBestSensor();

    /// Set the RCL internal estimate (called by Chassis::setPose to keep estimates
    /// synchronized)
    void setRclPose(const Pose& pose) { mLatestEstimate = pose; }

    // ========================================================================
    // Accessors
    // ========================================================================

    const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const { return mSensors; }
    Pose latestEstimate() const { return mLatestEstimate; }
    int activeSensorCount() const { return mActiveSensorCount; }

private:
    // ========================================================================
    // Ray intersection helper
    // ========================================================================

    // Note: field wall geometry is defined in Util.hpp (Field::FIELD_HALF_WALL).
    // getBotCoordFromSensor() determines which wall the ray hits and derives
    // the constrained coordinate (X from east/west walls, Y from north/south).

    /// Line-line intersection: ray from (rx,ry) with direction (cos,sin) × line segment
    /// Returns distance along ray, or maxRange if no intersection
    float intersectLineSegment(float rx, float ry, float rCos, float rSin,
                                float x1, float y1, float x2, float y2,
                                float maxRange) const;

    // ========================================================================
    // Background task
    // ========================================================================

    static void taskLoop(void* param);
    void run();

    // ========================================================================
    // Members
    // ========================================================================

    Config mConfig;
    OdomTracking& mOdom;
    std::array<RclSensor, MAX_DISTANCE_SENSORS> mSensors;
    std::vector<LineObstacle> mLineObstacles;
    std::vector<CircleObstacle> mCircleObstacles;

    Pose mLatestEstimate;
    int mActiveSensorCount{0};

    // Timed disable support
    std::array<float, MAX_DISTANCE_SENSORS> mDisableTimers{};

    pros::Task* mTask{nullptr};
    bool mRunning{false};
};

}  // namespace FBLIB
