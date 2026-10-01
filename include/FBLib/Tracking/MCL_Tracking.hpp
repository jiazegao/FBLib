#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "FBLib/Tracking/Odom_Tracking.hpp"
#include "FBLib/Util/FastTrig.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// MclTracking — Monte Carlo Localization via particle filter
// ============================================================================
//
// Uses 1024 particles with a Gaussian sensor model to estimate the robot's
// absolute position on the field. Particles are propagated using odometry
// (IMU + tracking wheels) and weighted by comparing ray-cast distances
// against actual V5 distance sensor readings.
//
// Typical usage:
//   MclTracking mcl(odomSensors, distanceSensors, startPose);
//   mcl.startTracking();  // begins background task @ ~40 Hz
//   // ...during autonomous...
//   Pose est = mcl.getEstimate().first;
// ============================================================================

class MclTracking {
public:
    // ========================================================================
    // Configuration
    // ========================================================================

    struct Config {
        int particleCount = 1024;
        int resampleThreshold = 512;      // N_eff below this triggers resample
        float maxSensorRange = 100.0f;     // inches
        float distResampleVariance = 1.5f; // inches, jitter added during resample
        float thetaResampleVariance = 0.01f;  // radians
        float maxThetaDeviation = 0.08f;   // radians, clamp particle theta vs IMU

        // Per-sensor-type noise multipliers.
        // These scale the standard normal noise applied to each axis's motion delta.
        // Different sensor types have different noise characteristics:
        //   - Tracking wheels: direct ground contact, low slip
        //   - IME (motor encoders): wheel slip, gear backlash → higher variance
        //   - No sensor: pure random walk, no measurement at all
        float trackingWheelVariance = 0.08f;   // dedicated tracking (dead) wheels
        float imeVariance = 0.12f;             // integrated motor encoders (more slip)
        float imuVariance = 0.005f;

        // Horizontal noise when no lateral sensor exists.
        // Without a horizontal tracking wheel, lateral position is unmeasured.
        // Two noise terms combine:
        //   1. Movement-proportional: lateral slip scales with forward travel
        //   2. Constant: baseline random walk each update
        float horizDependentVarianceProp = 0.03f;  // multiplied by |dVert|
        float horizConstantNoise = 0.03f;          // inches/update baseline

        float distSyncProp = 0.10f;        // lerp factor for syncing position to chassis
        float thetaSyncProp = 0.001f;      // unused: heading is never synced (IMU is authoritative)
        float updatePeriodMs = 25.0f;      // background task interval
        int confidenceThreshold = 10;      // min sensor confidence
        float minSensorRange = 1.0f;       // mm — readings below this are invalid
                                           //   (increase if sensors are recessed)
        bool autoSync = true;              // automatically sync pose to odometry
    };

    // ========================================================================
    // Compile-time constants (must be before first use as template arguments)
    // ========================================================================

    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;
    static constexpr int PARTICLE_MAX = 1024;
    static constexpr int NOISE_POOL_SIZE = 1024;
    static constexpr int NOISE_MASK = NOISE_POOL_SIZE - 1;
    static constexpr int GAUSSIAN_LUT_SIZE = 1024;

    // ========================================================================
    // Field geometry types (used for ray-casting)
    // ========================================================================

    // `sensors` says which distance sensors the element applies to: bit i =
    // sensor slot i. Clear the bits of sensors whose beams pass over or under
    // it (e.g. a low goal base under high-mounted sensors).
    struct LineObstacle {
        Pose p1, p2;
        uint8_t sensors = 0xFF;
    };

    struct CircleObstacle {
        float x, y, radius;
        uint8_t sensors = 0xFF;
    };

    // ========================================================================
    // Constructor / Destructor
    // ========================================================================

    MclTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);

    ~MclTracking();

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Start the background tracking task
    void startTracking();

    /// Stop the background tracking task
    void stopTracking();

    /// Check if tracking task is running
    bool isTracking() const;

    // ========================================================================
    // Core MCL operations
    // ========================================================================

    /// Motion update — propagate particles using decomposed odometry delta.
    /// Reads the per-axis (vertical, horizontal, angular) delta accumulated
    /// by OdomTracking::consumeDelta().  dVert and dHoriz are independent
    /// sensor measurements, allowing statistically correct per-axis noise.
    void predict();

    /// Sensor update — weight particles by how well they match sensor readings
    void updateWeights();

    /// Systematic low-variance resampling
    void resample();

    /// Get weighted-average pose estimate and effective sample size
    /// Returns {estimatedPose, N_eff}. N_eff below threshold triggers resample.
    std::pair<Pose, float> getEstimate() const;

    // ========================================================================
    // Pose management
    // ========================================================================

    /// Set the robot pose (reinitializes particles around this pose)
    void setPose(const Pose& pose);

    /// Reset all particles uniformly across the field
    void uniformReset();

    /// Run one full MCL iteration (predict + updateWeights + resample if needed + sync)
    /// Returns the new estimated pose
    Pose update();

    /// Smoothly pull the odometry POSITION toward the MCL estimate. Heading is
    /// left alone — the IMU is authoritative for orientation.
    void syncToOdometry();

    // ========================================================================
    // Sensor management (8 sensors, indexed 0-7)
    // ========================================================================

    /// Replace all distance sensors (e.g. after construction)
    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    /// Set sensor mount offsets (robot-frame: x=forward, y=left, theta=pointing angle).
    /// Call once after construction with your robot's measured sensor positions.
    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);

    /// Enable a specific sensor
    void enableSensor(int index);

    /// Disable a specific sensor
    void disableSensor(int index);

    /// Temporarily disable a sensor for a duration (milliseconds)
    void disableSensorFor(int index, float durationMs);

    // ========================================================================
    // Obstacles (for masking out known field elements)
    // ========================================================================

    /// Set this season's field elements: whatever inside the walls a distance
    /// sensor can see (goal bases, match loaders, ...). Particles expect a
    /// reading to end at the nearest element or wall; the walls are built in.
    void setFieldElements(const std::vector<LineObstacle>& lines,
                          const std::vector<CircleObstacle>& circles = {});

    /// Set disabling obstacles: a reading whose ray crosses one is ignored
    /// (e.g. to keep sensors off the far side of the field). The vectors must
    /// outlive the tracking.
    void setObstacles(const std::vector<LineObstacle>* lineObstacles = nullptr,
                      const std::vector<CircleObstacle>* circleObstacles = nullptr);

    /// Set tracking wheel drift compensation
    void setDrift(float verticalDrift, float horizontalDrift);

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get the raw MCL estimate (before sync)
    Pose getRawEstimate() const;

    /// Get the distance sensor readings (inches), index 0-7
    const std::array<float, SENSOR_COUNT>& sensorReadings() const { return mSensorReadingsInch; }

    /// Get sensor validity flags
    const std::array<bool, SENSOR_COUNT>& validSensors() const { return mValidSensors; }

    /// Set the distance sync proportion
    void setDistSyncProp(float prop) { mConfig.distSyncProp = prop; }

private:
    // ========================================================================
    // Internal types
    // ========================================================================

    struct Particle {
        Pose pose;
        float weight = 1.0f;
    };

    struct Trig {
        float cosVal, sinVal;
    };

    // ========================================================================
    // Ray-casting helpers
    // ========================================================================

    /// Intersect a ray with a line segment; returns distance or maxRange if no hit
    float intersectLine(float rayX, float rayY, const LineObstacle& wall,
                        float maxRange, float rayCos, float raySin) const;

    /// Intersect a ray with a circle; returns distance or maxRange if no hit
    float intersectCircle(float rayX, float rayY, const CircleObstacle& circle,
                          float maxRange, float dx, float dy) const;

    /// Cast a ray from a particle through a sensor and find nearest obstacle
    float castRay(const Particle& particle, int sensorIndex) const;

    /// Build the Gaussian probability LUT (sigma is scaled by sensor angle)
    void buildGaussianLUT();

    // ========================================================================
    // Noise generation
    // ========================================================================

    float nextNoise();
    void regenerateNoise();

    // ========================================================================
    // Unlocked implementations (caller holds mMutex)
    // ========================================================================

    void initParticlesAround(const Pose& pose);
    void scatterParticles();
    void predictUnlocked();
    void updateWeightsUnlocked();
    void resampleUnlocked();
    std::pair<Pose, float> estimateUnlocked() const;
    void syncUnlocked();

    // ========================================================================
    // Background task
    // ========================================================================

    static void taskLoop(void* param);
    void run();

    // ========================================================================
    // Member variables
    // ========================================================================

    Config mConfig;
    OdomTracking& mOdom;
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;

    // Sensor state
    std::array<bool, SENSOR_COUNT> mValidSensors{};
    std::array<int, SENSOR_COUNT> mSensorReadingsMm{};
    std::array<float, SENSOR_COUNT> mSensorReadingsInch{};
    std::array<int, SENSOR_COUNT> mSensorConfs{};
    std::array<bool, SENSOR_COUNT> mDisabledSensors{};
    std::array<float, SENSOR_COUNT> mDisableTimers{};

    // Season field elements (setFieldElements)
    std::vector<LineObstacle> mFieldLines;
    std::vector<CircleObstacle> mFieldCircles;

    // Disabling obstacle pointers (externally managed)
    const std::vector<LineObstacle>* mLineObstacles{nullptr};
    const std::vector<CircleObstacle>* mCircleObstacles{nullptr};

    // Particles (double-buffered for resampling)
    std::array<Particle, PARTICLE_MAX> mParticlesA;
    std::array<Particle, PARTICLE_MAX> mParticlesB;
    std::array<Particle, PARTICLE_MAX>* mParticles{&mParticlesA};
    std::array<Particle, PARTICLE_MAX>* mNewParticles{&mParticlesB};


    // Gaussian LUT
    alignas(64) float mGaussianLUT[GAUSSIAN_LUT_SIZE]{};

    // Sensor mount poses (robot-frame offsets + pointing angles)
    std::array<Pose, SENSOR_COUNT> mSensorMounts{};
    std::array<Trig, SENSOR_COUNT> mMountTrigs{};

    // Noise pool
    alignas(64) std::array<float, NOISE_POOL_SIZE> mNoisePool{};
    int mNoiseIdx{0};
    int mRegenIdx{0};

    // RNG
    std::mt19937 mGen;
    std::normal_distribution<float> mDist;

    // Drift compensation (per-update values — setDrift divides by update rate)
    float mVertDrift{0.0f};
    float mHorizDrift{0.0f};

    // State
    Pose mLastResamplePose{};
    Pose mRawEstimate{};
    float mLatestSpeed{0.0f};

    // Guards particles, sensor configuration and estimate state: the MCL task
    // and user tasks (setPose, sensor/obstacle changes) touch them
    // concurrently. Lock order: this mutex, then OdomTracking's.
    mutable pros::Mutex mMutex;

    // Background task.
    // mRunning is atomic: it's written by stopTracking() (caller task) and
    // read by the loop in run() (worker task) — a cross-task flag.
    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};

}  // namespace FBLIB
