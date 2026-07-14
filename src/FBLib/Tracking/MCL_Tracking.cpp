#include "FBLib/Tracking/MCL_Tracking.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Util/FastTrig.hpp"
#include "pros/llemu.hpp"

namespace FBLIB {

// ============================================================================
// Ray-casting helpers
// ============================================================================

float MclTracking::intersectLine(float rayX, float rayY,
                                 const LineObstacle& wall,
                                 float maxRange,
                                 float rayCos, float raySin) const {
    // Quick bound check
    float xMin = std::min(wall.p1.x, wall.p2.x);
    float xMax = std::max(wall.p1.x, wall.p2.x);
    float yMin = std::min(wall.p1.y, wall.p2.y);
    float yMax = std::max(wall.p1.y, wall.p2.y);

    if ((rayCos > 0.0f && rayX > xMax) ||
        (rayCos < 0.0f && rayX < xMin) ||
        (raySin > 0.0f && rayY > yMax) ||
        (raySin < 0.0f && rayY < yMin)) {
        return maxRange;
    }

    float x1 = wall.p1.x, y1 = wall.p1.y;
    float x2 = wall.p2.x, y2 = wall.p2.y;
    float x3 = rayX,       y3 = rayY;
    float x4 = rayX + rayCos * maxRange;
    float y4 = rayY + raySin * maxRange;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(den) < 1e-6f) return maxRange;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        return u * maxRange;
    }
    return maxRange;
}

float MclTracking::intersectCircle(float rayX, float rayY,
                                    const CircleObstacle& circle,
                                    float maxRange,
                                    float dx, float dy) const {
    // Bound check
    float xDiff = rayX - circle.x;
    float yDiff = rayY - circle.y;
    float cTemp = maxRange + circle.radius;
    if (xDiff * xDiff + yDiff * yDiff > cTemp * cTemp) {
        return maxRange;
    }

    float fx = rayX - circle.x;
    float fy = rayY - circle.y;
    float b = 2.0f * (fx * dx + fy * dy);
    float cVal = (fx * fx + fy * fy) - (circle.radius * circle.radius);
    float discriminant = b * b - 4.0f * cVal;

    if (discriminant < 0.0f) return maxRange;
    discriminant = std::sqrt(discriminant);

    float t1 = (-b - discriminant) * 0.5f;
    float t2 = (-b + discriminant) * 0.5f;

    if (t1 >= 0.0f && t1 <= maxRange) return t1;
    if (t2 >= 0.0f && t2 <= maxRange) return t2;
    return maxRange;
}

// ============================================================================
// Noise generation
// ============================================================================

float MclTracking::nextNoise() {
    mNoiseIdx = (mNoiseIdx + 123) & NOISE_MASK;
    return mNoisePool[mNoiseIdx];
}

void MclTracking::regenerateNoise() {
    mNoisePool[mRegenIdx] = mDist(mGen);
    mRegenIdx = (mRegenIdx + 1) & NOISE_MASK;
}

// ============================================================================
// Gaussian LUT
// ============================================================================

void MclTracking::buildGaussianLUT() {
    for (int i = 0; i < GAUSSIAN_LUT_SIZE; i++) {
        float x = (static_cast<float>(i) / GAUSSIAN_LUT_SIZE) * 4.0f;  // 0..4 sigmas
        mGaussianLUT[i] = std::exp(-(x * x) / 2.0f);
    }
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

MclTracking::MclTracking(OdomTracking& odom,
                         std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                         const Pose& startPose,
                         const Config& config)
    : mConfig(config),
      mOdom(odom),
      mDistanceSensors(distanceSensors),
      mGen(std::random_device{}()),
      mDist(0.0f, 1.0f) {
    
    // Check particle count valitity
    if (mConfig.particleCount > PARTICLE_MAX) {
        mConfig.particleCount = PARTICLE_MAX;
    }

    // Sensor mounts default to zero — call setSensorMounts() after
    // construction with your robot's measured sensor positions.
    mSensorMounts.fill({});
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mMountTrigs[i] = {1.0f, 0.0f};  // cos(0)=1, sin(0)=0
    }

    // Pre-generate noise pool
    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        mNoisePool[i] = mDist(mGen);
    }

    // Build Gaussian LUT
    buildGaussianLUT();

    // Initialize IMU reference
    mLastImuHeading = mOdom.imuHeadingRad();

    // Initialize particles around start pose
    setPose(startPose);
}

MclTracking::~MclTracking() {
    stopTracking();
}

// ============================================================================
// Lifecycle
// ============================================================================

void MclTracking::startTracking() {
    if (mTask == nullptr) {
        mRunning = true;
        mTask = new pros::Task([this]() { run(); });
    }
}

void MclTracking::stopTracking() {
    mRunning = false;
    if (mTask != nullptr) {
        // Wait for the task to exit on its own (it self-deletes when run()
        // returns). Hard-killing with remove() while the task holds the
        // odometry mutex (inside getPose/setPose) would deadlock every other
        // odometry consumer permanently — only remove() as a last resort.
        uint32_t start = pros::millis();
        while (pros::millis() - start < 200) {
            uint32_t state = mTask->get_state();
            if (state == pros::E_TASK_STATE_DELETED ||
                state == pros::E_TASK_STATE_INVALID) break;
            pros::delay(5);
        }
        uint32_t state = mTask->get_state();
        if (state != pros::E_TASK_STATE_DELETED &&
            state != pros::E_TASK_STATE_INVALID) {
            mTask->remove();
        }
        delete mTask;
        mTask = nullptr;
    }
}

bool MclTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void MclTracking::run() {
    while (mRunning) {
        uint32_t startTime = pros::millis();
        // Odometry freshness is provided by the Chassis background odometry
        // task (the sole caller of mOdom.update()). Calling update() here too
        // would race on odometry state and consume delta baselines.
        update();

        // Maintain consistent update rate
        int32_t elapsed = pros::millis() - startTime;
        int32_t remaining = static_cast<int32_t>(mConfig.updatePeriodMs) - elapsed;
        if (remaining > 2) {
            pros::delay(remaining);
        } else {
            pros::delay(2);  // Minimum delay to yield
        }
    }
}

// ============================================================================
// Pose management
// ============================================================================

void MclTracking::setPose(const Pose& pose) {
    mLastImuHeading = mOdom.imuHeadingRad();

    // Discard motion accumulated before this reposition — it predates the
    // new particle distribution and would otherwise be applied to freshly
    // initialized particles on the next predict().
    mOdom.consumeDelta();

    std::normal_distribution<float> xDist(pose.x, mConfig.distResampleVariance);
    std::normal_distribution<float> yDist(pose.y, mConfig.distResampleVariance);
    std::normal_distribution<float> tDist(pose.theta, mConfig.thetaResampleVariance);

    auto& particles = *mParticles;
    for (auto& p : particles) {
        p.pose = {xDist(mGen), yDist(mGen), tDist(mGen)};
        p.weight = 1.0f;
    }

    mRawEstimate = pose;
    mLastResamplePose = pose;
    mLatestSpeed = 0.0f;
}

void MclTracking::uniformReset() {
    mLastImuHeading = mOdom.imuHeadingRad();

    std::uniform_real_distribution<float> xDist(-Field::FIELD_HALF_WALL, Field::FIELD_HALF_WALL);
    std::uniform_real_distribution<float> yDist(-Field::FIELD_HALF_WALL, Field::FIELD_HALF_WALL);
    std::normal_distribution<float> tDist(0.0f, PI);

    auto& particles = *mParticles;
    for (auto& p : particles) {
        p.pose = {xDist(mGen), yDist(mGen), tDist(mGen)};
        p.weight = 1.0f;
    }

    mRawEstimate = Pose{};
    mLastResamplePose = Pose{};
    mLatestSpeed = 0.0f;
}

// ============================================================================
// predict() — motion update (propagate particles using odometry)
// ============================================================================

void MclTracking::predict() {
    // ========================================================================
    // Motion update using decomposed odometry delta with per-sensor-type noise.
    //
    // dVert and dHoriz are independent sensor measurements from OdomTracking.
    // Three sensor sources → three configurable noise levels:
    //
    //   Vertical axis (always measured):
    //     Tracking wheel → trackingWheelVariance (low, direct ground contact)
    //     Motor encoders → imeVariance             (higher, wheel slip/backlash)
    //
    //   Horizontal axis:
    //     Tracking wheel → trackingWheelVariance   (low, direct measurement)
    //     No sensor      → random walk:
    //         horizDependentVarianceProp * |dVert| (lateral slip ∝ forward travel)
    //         + horizConstantNoise                (baseline per update)
    //
    // The decomposed delta from odometry preserves per-axis noise independence
    // because dVert and dHoriz are physically distinct sensor channels, not
    // trigonometrically decomposed from a fused global (X,Y) pose.
    // ========================================================================

    // consumeDelta() returns motion ACCUMULATED since our previous predict()
    // and resets the accumulator (thread-safe). getLastDelta() must not be
    // used here: it holds only the most recent 10ms odometry tick, so any
    // ticks that ran between MCL iterations (odometry updates at 10ms, MCL
    // at 25ms) would be silently lost and particles would fall behind.
    OdomDelta delta = mOdom.consumeDelta();

    float currentImuHeadingRad = mOdom.imuHeadingRad();
    float dThetaRad = delta.dTheta;

    // Update raw estimate heading
    mRawEstimate.theta += dThetaRad;
    mRawEstimate.theta = wrapRad(mRawEstimate.theta);

    // Arc approximation scale factor (sin(x)/x for small x → 1)
    float halfDThetaRad = dThetaRad * 0.5f;
    float moveScale;
    if (std::fabs(dThetaRad) < 1e-6f) {
        moveScale = 1.0f;
    } else {
        moveScale = std::sin(halfDThetaRad) / halfDThetaRad;
    }

    // Apply per-axis drift compensation.
    // Drift is stored per-update (setDrift divides by updates/sec).
    float dVertPure  = delta.dVert  + mVertDrift;
    float dHorizPure = delta.dHoriz + mHorizDrift;

    // Speed estimate (inches/sec) for resampling gating
    mLatestSpeed = std::hypot(dVertPure, dHorizPure) * (1000.0f / mConfig.updatePeriodMs);

    // Combined drift variance for particle jitter
    float driftVariance = std::hypot(mVertDrift, mHorizDrift) * 0.5f;

    // ========================================================================
    // Per-sensor-type noise model.
    //
    // Three sensor sources → three noise levels:
    //   Tracking wheel: low noise (direct ground contact, minimal slip)
    //   IME (motor encoder): higher noise (wheel slip, gear backlash)
    //   None: pure random walk (no measurement — lateral axis only)
    //
    // The vertical axis always has SOME sensor (tracking wheel or IME).
    // The horizontal axis may have a tracking wheel or nothing at all
    // (tank drives cannot measure lateral movement from motor encoders).
    // ========================================================================

    bool hasVertWheel  = mOdom.hasVerticalTrackingWheel();
    bool hasHorizWheel = mOdom.hasHorizontalTracking();

    // Select vertical noise: tracking wheel → low, IME → higher
    float vertVariance = hasVertWheel ? mConfig.trackingWheelVariance : mConfig.imeVariance;

    auto& particles = *mParticles;
    for (int i = 0; i < mConfig.particleCount; i++) {
        auto& p = particles[i];

        // — Vertical axis (always measured: tracking wheel or IME) —
        float vertNoise = 1.0f + nextNoise() * vertVariance;
        float forwardDist = dVertPure * vertNoise + nextNoise() * driftVariance;

        // — Horizontal axis —
        float localHoriz;
        if (hasHorizWheel) {
            // Dedicated horizontal tracking wheel: low-noise measurement.
            // Noise scales with measured lateral distance.
            float horizNoise = 1.0f + nextNoise() * mConfig.trackingWheelVariance;
            float strafeDist = dHorizPure * horizNoise + nextNoise() * driftVariance;
            localHoriz = strafeDist * moveScale;
        } else {
            // No horizontal sensor: pure random walk with two components:
            //   1. Movement-coupled: lateral slip ∝ forward travel (goes through
            //      the arc approximation — it's tied to physical motion).
            //   2. Constant: baseline uncertainty per update (independent of
            //      movement geometry, does NOT go through arc approximation).
            float movementNoise = nextNoise() * driftVariance
                                + nextNoise() * mConfig.horizDependentVarianceProp * std::fabs(dVertPure);
            localHoriz = movementNoise * moveScale
                       + nextNoise() * mConfig.horizConstantNoise;
        }

        float localVert = forwardDist * moveScale;

        // Update particle theta with IMU noise
        p.pose.theta += halfDThetaRad + nextNoise() * mConfig.imuVariance;

        float pCos = FastTrig::cos(p.pose.theta);
        float pSin = FastTrig::sin(p.pose.theta);

        // Transform local-frame motion to global frame using the
        // particle's own heading (each particle has a different heading,
        // so each transforms the same local delta differently).
        p.pose.x += localVert * pCos + localHoriz * pSin;
        p.pose.y += localVert * pSin - localHoriz * pCos;

        // Add second half of rotation and clamp to IMU heading.
        // Particles cannot deviate arbitrarily far from the IMU —
        // the IMU is the ground-truth for orientation.
        // Use angular-distance clamping instead of raw linear clamp —
        // linear clamp breaks near the ±π wrap boundary where a particle's
        // angle may be on the opposite side of the boundary from the IMU.
        {
            float rawTheta = p.pose.theta + halfDThetaRad;
            float diff = angleDiffRad(currentImuHeadingRad, rawTheta);
            float clampedDiff = clamp(diff, -mConfig.maxThetaDeviation, mConfig.maxThetaDeviation);
            p.pose.theta = wrapRad(currentImuHeadingRad + clampedDiff);
        }
    }

    mLastImuHeading = currentImuHeadingRad;
}

// ============================================================================
// updateWeights() — sensor update
// ============================================================================

void MclTracking::updateWeights() {
    float activeSensors = 0.0f;

    // Current robot trig (from raw estimate)
    float currCos = FastTrig::cos(mRawEstimate.theta);
    float currSin = FastTrig::sin(mRawEstimate.theta);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Null check — skip unused sensor slots
        if (mDistanceSensors[i] == nullptr) {
            mValidSensors[i] = false;
            continue;
        }

        // Read sensor
        mSensorReadingsMm[i] = mDistanceSensors[i]->get();
        mSensorReadingsInch[i] = mSensorReadingsMm[i] * MM_TO_INCH;
        mSensorConfs[i] = mDistanceSensors[i]->get_confidence();
        mValidSensors[i] = true;

        // Compute world-space sensor position from raw estimate
        float sx = mRawEstimate.x + (mSensorMounts[i].x * currCos - mSensorMounts[i].y * currSin);
        float sy = mRawEstimate.y + (mSensorMounts[i].x * currSin + mSensorMounts[i].y * currCos);

        // Sensor ray direction
        float sCos = currCos * mMountTrigs[i].cosVal - currSin * mMountTrigs[i].sinVal;
        float sSin = currSin * mMountTrigs[i].cosVal + currCos * mMountTrigs[i].sinVal;

        // Validation checks
        if (mDisabledSensors[i]) { mValidSensors[i] = false; continue; }
        if (mSensorReadingsMm[i] > 9000) { mValidSensors[i] = false; continue; }
        if (mSensorReadingsMm[i] < mConfig.minSensorRange) { mValidSensors[i] = false; continue; }
        // Confidence threshold for longer readings
        if (mSensorReadingsMm[i] > 200 && mSensorConfs[i] < mConfig.confidenceThreshold) {
            mValidSensors[i] = false; continue;
        }

        // Check built-in disabling obstacles (modify kDisablingLines per season)
        {
            bool disabled = false;
            for (const auto& line : kDisablingLines) {
                if (intersectLine(sx, sy, line, mConfig.maxSensorRange, sCos, sSin) < mConfig.maxSensorRange) {
                    disabled = true;
                    break;
                }
            }
            if (disabled) { mValidSensors[i] = false; continue; }
        }

        // Check user-supplied disabling obstacles
        if (mLineObstacles != nullptr) {
            for (const auto& line : *mLineObstacles) {
                if (intersectLine(sx, sy, line, mConfig.maxSensorRange, sCos, sSin) < mConfig.maxSensorRange) {
                    mValidSensors[i] = false;
                    break;
                }
            }
        }
        if (!mValidSensors[i]) continue;

        if (mCircleObstacles != nullptr) {
            for (const auto& circle : *mCircleObstacles) {
                if (intersectCircle(sx, sy, circle, mConfig.maxSensorRange, sCos, sSin) < mConfig.maxSensorRange) {
                    mValidSensors[i] = false;
                    break;
                }
            }
        }
        if (!mValidSensors[i]) continue;

        activeSensors += 1.0f;
    }

    // Compute per-sensor sigma (measurement noise estimate)
    float invSigmas[SENSOR_COUNT];
    float sensorCountMult = (activeSensors > 0.0f) ? std::sqrt(activeSensors * 0.25f) : 1.0f;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (!mValidSensors[i]) continue;

        // Angle-dependent sigma scaling (sensors are less accurate at glancing angles)
        float angleOffset = std::fmod(std::fabs(mRawEstimate.theta + mSensorMounts[i].theta), HALF_PI);
        if (angleOffset > QUARTER_PI) {
            angleOffset = HALF_PI - angleOffset;
        }
        float angleMultiplier = 1.0f + angleOffset * (2.0f / PI * 1.2f);

        // Base sigma: close readings have fixed noise, far readings have proportional noise
        float sigma = (mSensorReadingsMm[i] <= 200)
            ? 0.787f   // ~20mm for close readings
            : mSensorReadingsInch[i] * 0.05f;  // 5% of reading for far readings

        // Apply angle-dependent scaling (sensors are less accurate at glancing angles)
        sigma *= angleMultiplier;

        // Confidence scaling for far readings
        if (mSensorReadingsMm[i] > 200) {
            float safeConf = std::max(static_cast<float>(mSensorConfs[i]), 1.0f);
            sigma *= 32.0f / safeConf;
        }

        sigma *= sensorCountMult;
        invSigmas[i] = 1.0f / sigma;
    }

    // Weight particles
    auto& particles = *mParticles;
    for (int i = 0; i < mConfig.particleCount; i++) {
        float totalWeight = 1.0f;
        auto& p = particles[i];

        // Instant disqualification if outside field bounds
        if (std::fabs(p.pose.x) > Field::FIELD_HALF_WALL ||
            std::fabs(p.pose.y) > Field::FIELD_HALF_WALL) {
            p.weight = 1e-35f;
            continue;
        }

        float pCos = FastTrig::cos(p.pose.theta);
        float pSin = FastTrig::sin(p.pose.theta);

        for (int j = 0; j < SENSOR_COUNT; j++) {
            if (!mValidSensors[j]) continue;

            // Sensor ray from particle position.
            // Rotate the sensor mount offset by the PARTICLE's heading
            // (each particle has a different orientation).
            float sCos = pCos * mMountTrigs[j].cosVal - pSin * mMountTrigs[j].sinVal;
            float sSin = pSin * mMountTrigs[j].cosVal + pCos * mMountTrigs[j].sinVal;

            float sx = p.pose.x + (mSensorMounts[j].x * pCos - mSensorMounts[j].y * pSin);
            float sy = p.pose.y + (mSensorMounts[j].x * pSin + mSensorMounts[j].y * pCos);

            // Ray cast from particle.
            // Priority order: line targets → circle targets → field walls.
            // The first object hit determines the expected distance.
            float pDist = mConfig.maxSensorRange;

            // 1) Non-disabling line targets (goal legs, barriers, etc.)
            for (const auto& target : kFieldTargets) {
                float dist = intersectLine(sx, sy, target, mConfig.maxSensorRange, sCos, sSin);
                if (dist < pDist) pDist = dist;
            }

            // 2) Non-disabling circle targets (match loaders, etc.)
            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                for (const auto& circle : kFieldCircles) {
                    float dist = intersectCircle(sx, sy, circle, mConfig.maxSensorRange, sCos, sSin);
                    if (dist < pDist) pDist = dist;
                }
            }

            // 3) Field walls (always present, lowest priority)
            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                for (const auto& wall : kFieldWalls) {
                    float dist = intersectLine(sx, sy, wall, mConfig.maxSensorRange, sCos, sSin);
                    if (dist < pDist) pDist = dist;
                }
            }

            // If no object hit at max range, weight is very low
            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                totalWeight *= 0.001f;
                continue;
            }

            // Compare expected vs actual distance using Gaussian LUT
            float z = std::fabs(mSensorReadingsInch[j] - pDist) * invSigmas[j];
            int lutIdx = static_cast<int>(z * 256.0f);

            if (lutIdx < GAUSSIAN_LUT_SIZE) {
                totalWeight *= mGaussianLUT[lutIdx];
            } else {
                totalWeight *= mGaussianLUT[GAUSSIAN_LUT_SIZE - 1] * 0.01f;
            }
        }
        p.weight = totalWeight;
    }
}

// ============================================================================
// resample() — systematic low-variance resampling
// ============================================================================

void MclTracking::resample() {
    auto& particles = *mParticles;
    auto& newParticles = *mNewParticles;

    // Calculate total weight
    float totalWeight = 0.0f;
    for (int i = 0; i < mConfig.particleCount; i++) {
        totalWeight += particles[i].weight;
    }

    // If all weights are zero, do uniform reset
    if (totalWeight < 1e-20f) {
        uniformReset();
        return;
    }

    float step = totalWeight / static_cast<float>(mConfig.particleCount);

    // Random start in [0, step)
    std::uniform_real_distribution<float> starter(0.0f, step);
    float currPos = starter(mGen);

    float cumWeight = particles[0].weight;
    int index = 0;

    for (int m = 0; m < mConfig.particleCount; m++) {
        // Walk to target position
        while (index < mConfig.particleCount - 1 && currPos > cumWeight) {
            index++;
            cumWeight += particles[index].weight;
        }

        // Select and jitter
        Particle selected = particles[index];
        selected.pose.x     += nextNoise() * mConfig.distResampleVariance;
        selected.pose.y     += nextNoise() * mConfig.distResampleVariance;
        selected.pose.theta  = wrapRad(selected.pose.theta +
                                       nextNoise() * mConfig.thetaResampleVariance);
        selected.weight = 1.0f;

        newParticles[m] = selected;
        currPos += step;
    }

    // Swap buffers
    std::swap(mParticles, mNewParticles);
}

// ============================================================================
// getEstimate() — weighted average of particles
// ============================================================================

std::pair<Pose, float> MclTracking::getEstimate() const {
    float xSum = 0.0f, ySum = 0.0f;
    float sinSum = 0.0f, cosSum = 0.0f;
    float totalWeight = 0.0f;
    float weightSqrSum = 0.0f;

    const auto& particles = *mParticles;
    for (int i = 0; i < mConfig.particleCount; i++) {
        const auto& p = particles[i];
        xSum += p.pose.x * p.weight;
        ySum += p.pose.y * p.weight;
        sinSum += FastTrig::sin(p.pose.theta) * p.weight;
        cosSum += FastTrig::cos(p.pose.theta) * p.weight;
        totalWeight += p.weight;
        weightSqrSum += p.weight * p.weight;
    }

    if (totalWeight < 1e-20f) {
        return {mRawEstimate, 0.0f};
    }

    Pose estimate = {
        xSum / totalWeight,
        ySum / totalWeight,
        std::atan2(sinSum, cosSum)
    };

    // N_eff = (sum of weights)^2 / sum of (weights^2)
    float nEff = totalWeight * totalWeight / weightSqrSum;

    return {estimate, nEff};
}

// ============================================================================
// update() — one full MCL iteration
// ============================================================================

Pose MclTracking::update() {
    // Update sensor disable timers
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (mDisableTimers[i] > 0.0f) {
            mDisableTimers[i] -= mConfig.updatePeriodMs;
            mDisabledSensors[i] = (mDisableTimers[i] > 0.0f);
        }
    }

    // predict() reads the decomposed delta from OdomTracking::getLastDelta().
    // The delta was stored during the mOdom.update() call in run() and is
    // purely physical (raw sensor deltas) — syncToOdometry() setPose calls
    // cannot contaminate it because mLastDelta is computed from accumulated
    // tracking-wheel distance differences, not from pose differences.
    predict();
    updateWeights();

    auto estimate = getEstimate();

    // Resample only when N_eff is too low AND robot has moved enough
    float distSinceResample = distanceToPoint(mLastResamplePose, estimate.first.x, estimate.first.y);

    bool shouldResample = false;
    if (estimate.second < mConfig.resampleThreshold * 0.5f) {
        shouldResample = true;
    } else if (estimate.second < mConfig.resampleThreshold &&
               distSinceResample > 5.0f &&
               mLatestSpeed < 100.0f) {
        shouldResample = true;
    }

    if (shouldResample) {
        resample();
        mLastResamplePose = estimate.first;
    }

    mRawEstimate = estimate.first;

    // Sync to odometry (modifies mOdom via setPose).
    // The delta used in predict() comes from OdomTracking::getLastDelta(),
    // which stores raw sensor deltas (tracking wheel distance differences).
    // These are unaffected by syncToOdometry() setPose calls — the sensor
    // readings don't change when the pose is corrected.  This keeps the
    // next iteration's motion delta free of sync artifacts.
    if (mConfig.autoSync) {
        syncToOdometry();
    }

    // Regenerate noise pool entries
    static constexpr int REGEN_COUNT = 3;
    for (int i = 0; i < REGEN_COUNT; i++) {
        regenerateNoise();
    }

    return estimate.first;
}

// ============================================================================
// syncToOdometry() — smooth interpolation of MCL into odometry
// ============================================================================

void MclTracking::syncToOdometry() {
    Pose odomPose = mOdom.getPose();

    // Lerp position
    float newX = odomPose.x + mConfig.distSyncProp * (mRawEstimate.x - odomPose.x);
    float newY = odomPose.y + mConfig.distSyncProp * (mRawEstimate.y - odomPose.y);

    // Lerp heading (with wrap)
    float delta = angleDiffRad(odomPose.theta, mRawEstimate.theta);
    float newTheta = odomPose.theta + mConfig.thetaSyncProp * delta;

    mOdom.setPose({newX, newY, wrapRad(newTheta)});

    // DO NOT adjust mLastImuHeading here — it tracks the raw IMU reading,
    // not the odometry heading.  Unlike the reference (which reads the chassis
    // pose for heading), we read mOdom.imuHeadingRad() in predict().  The raw
    // IMU value is unaffected by odometry syncs, so shifting mLastImuHeading
    // would introduce a spurious rotation on the next predict() call.
}

// ============================================================================
// Sensor enable/disable
// ============================================================================

void MclTracking::setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors) {
    mDistanceSensors = sensors;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mDisabledSensors[i] = (sensors[i] == nullptr);
        mDisableTimers[i] = 0.0f;
    }
}

void MclTracking::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    mSensorMounts = mounts;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mMountTrigs[i] = {std::cos(mSensorMounts[i].theta),
                          std::sin(mSensorMounts[i].theta)};
    }
}

void MclTracking::enableSensor(int index) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = false;
    mDisableTimers[index] = 0.0f;
}

void MclTracking::disableSensor(int index) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = true;
    mDisableTimers[index] = 1e20f;
}

void MclTracking::disableSensorFor(int index, float durationMs) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = true;
    mDisableTimers[index] = durationMs;
}

// ============================================================================
// Obstacle and drift management
// ============================================================================

void MclTracking::setObstacles(const std::vector<LineObstacle>* lineObstacles,
                                const std::vector<CircleObstacle>* circleObstacles) {
    mLineObstacles = lineObstacles;
    mCircleObstacles = circleObstacles;
}

void MclTracking::setDrift(float verticalDrift, float horizontalDrift) {
    // Convert per-second drift to per-update drift
    float updatesPerSec = 1000.0f / mConfig.updatePeriodMs;
    mVertDrift = verticalDrift / updatesPerSec;
    mHorizDrift = horizontalDrift / updatesPerSec;
}

// ============================================================================
// Accessors
// ============================================================================

Pose MclTracking::getRawEstimate() const {
    return mRawEstimate;
}

}  // namespace FBLIB
