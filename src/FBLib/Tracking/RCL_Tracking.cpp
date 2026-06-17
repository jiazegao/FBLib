#include "FBLib/Tracking/RCL_Tracking.hpp"

#include <algorithm>
#include <cmath>

namespace FBLIB {

// ============================================================================
// Ray-line intersection
// ============================================================================

float RclTracking::intersectLineSegment(float rx, float ry,
                                         float rCos, float rSin,
                                         float x1, float y1,
                                         float x2, float y2,
                                         float maxRange) const {
    float xMin = std::min(x1, x2);
    float xMax = std::max(x1, x2);
    float yMin = std::min(y1, y2);
    float yMax = std::max(y1, y2);

    if ((rCos > 0.0f && rx > xMax) || (rCos < 0.0f && rx < xMin) ||
        (rSin > 0.0f && ry > yMax) || (rSin < 0.0f && ry < yMin)) {
        return maxRange;
    }

    float x3 = rx, y3 = ry;
    float x4 = rx + rCos * maxRange;
    float y4 = ry + rSin * maxRange;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(den) < 1e-6f) return maxRange;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        return u * maxRange;
    }
    return maxRange;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

RclTracking::RclTracking(OdomTracking& odom, const Config& config)
    : mConfig(config), mOdom(odom) {}

RclTracking::~RclTracking() {
    stopTracking();
}

// ============================================================================
// Sensor management
// ============================================================================

void RclTracking::setSensor(int index, pros::Distance* sensor, const Pose& mountOffset) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].sensor = sensor;
    mSensors[index].mountOffset = mountOffset;
    mSensors[index].enabled = true;
}

void RclTracking::configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                                     const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts) {
    int activeCount = 0;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (sensors[i] != nullptr) {
            mSensors[i].sensor       = sensors[i];
            mSensors[i].mountOffset  = mounts[i];
            mSensors[i].enabled      = true;
            mDisableTimers[i]        = 0.0f;
            activeCount++;
        } else {
            mSensors[i].sensor       = nullptr;
            mSensors[i].mountOffset  = {};
            mSensors[i].enabled      = false;
            mDisableTimers[i]        = 0.0f;
        }
    }
    // Clamp to [1, MAX_DISTANCE_SENSORS] so loop bounds remain safe
    mConfig.sensorCount = (activeCount > 0) ? activeCount : 1;
}

void RclTracking::enableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = true;
}

void RclTracking::disableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = false;
}

void RclTracking::disableSensorFor(int index, float durationMs) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = false;
    mDisableTimers[index] = durationMs;
}

// ============================================================================
// Obstacle management
// ============================================================================

void RclTracking::addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs) {
    mLineObstacles.push_back({x1, y1, x2, y2, lifetimeMs});
}

void RclTracking::addCircleObstacle(float x, float y, float radius, float lifetimeMs) {
    mCircleObstacles.push_back({x, y, radius, lifetimeMs});
}

void RclTracking::clearObstacles() {
    mLineObstacles.clear();
    mCircleObstacles.clear();
}

// ============================================================================
// updateSensorPoses()
// ============================================================================

void RclTracking::updateSensorPoses(const Pose& robotPose) {
    float rCos = std::cos(robotPose.theta);
    float rSin = std::sin(robotPose.theta);

    for (int i = 0; i < mConfig.sensorCount; i++) {
        auto& s = mSensors[i];
        if (!s.enabled || s.sensor == nullptr) {
            s.valid = false;
            continue;
        }

        // Rotate mount offset into world frame
        s.worldX = robotPose.x + (s.mountOffset.x * rCos - s.mountOffset.y * rSin);
        s.worldY = robotPose.y + (s.mountOffset.x * rSin + s.mountOffset.y * rCos);

        // Sensor pointing direction in world frame
        float sensorWorldAngle = robotPose.theta + s.mountOffset.theta;
        s.worldHeading = sensorWorldAngle;
        s.rayCos = std::cos(sensorWorldAngle);
        s.raySin = std::sin(sensorWorldAngle);

        // Read sensor
        int readingMm = s.sensor->get();
        s.readingInch = readingMm * MM_TO_INCH;
        s.confidence = s.sensor->get_confidence();
    }
}

// ============================================================================
// isValidReading()
// ============================================================================

bool RclTracking::isValidReading(int index) const {
    const auto& s = mSensors[index];
    if (!s.enabled || s.sensor == nullptr) return false;

    // — Range checks —
    // V5 distance sensor practical maximum is ~2000 mm (~78.7 in). Readings
    // above this are unreliable; above 9000 is the "no object" sentinel.
    // Use the stored reading from updateSensorPoses() to avoid a TOCTOU race
    // where the sensor value changes between the range check here and the
    // coordinate computation in getBotCoordFromSensor().
    float readingInch = s.readingInch;
    if (readingInch > 2000.0f * MM_TO_INCH || readingInch < 1.0f * MM_TO_INCH) return false;

    // — Confidence check —
    // V5 sensor confidence is 0–63. Low-confidence readings produce noise.
    if (readingInch > 200.0f * MM_TO_INCH && s.confidence < mConfig.confidenceThreshold) return false;

    // — Heading alignment check —
    // RCL requires the sensor ray to be near-orthogonal to a field wall
    // (within angleTolerance of 0°, 90°, 180°, or 270° in world frame).
    // A diagonal ray cannot determine which wall was hit and the distance
    // measurement has amplified angle error.
    float angleTolRad = degToRad(mConfig.angleTolerance);
    float angleMod = std::fmod(std::fabs(s.worldHeading), HALF_PI);
    if (angleMod > angleTolRad && angleMod < HALF_PI - angleTolRad) {
        return false;
    }

    // Check if ray hits a known obstacle before the wall
    for (const auto& obs : mLineObstacles) {
        float dist = intersectLineSegment(s.worldX, s.worldY, s.rayCos, s.raySin,
                                           obs.x1, obs.y1, obs.x2, obs.y2, 200.0f);
        if (dist < 200.0f && dist < readingInch) return false;
    }

    // Check circle obstacles
    for (const auto& obs : mCircleObstacles) {
        float fx = s.worldX - obs.x;
        float fy = s.worldY - obs.y;
        float b = 2.0f * (fx * s.rayCos + fy * s.raySin);
        float c = fx * fx + fy * fy - obs.radius * obs.radius;
        float disc = b * b - 4.0f * c;

        if (disc >= 0.0f) {
            disc = std::sqrt(disc);
            float t1 = (-b - disc) * 0.5f;
            float t2 = (-b + disc) * 0.5f;
            float tMin = std::min(t1, t2);
            if (tMin >= 0.0f && tMin < readingInch) return false;
        }
    }

    return true;
}

// ============================================================================
// getBotCoordFromSensor()
// ============================================================================

std::pair<CoordType, float> RclTracking::getBotCoordFromSensor(int index) const {
    // ========================================================================
    // Derive robot coordinate from a single sensor intersecting a field wall.
    //
    // A distance sensor pointed at a wall determines ONE spatial dimension:
    //   • east/west wall → X coordinate
    //   • north/south wall → Y coordinate
    //
    // The unconstrained coordinate is NOT returned — it must come from
    // another sensor or from odometry.  This is physically necessary:
    // a single ray-to-wall distance cannot constrain the axis parallel
    // to the wall.
    // ========================================================================
    const auto& s = mSensors[index];
    if (!isValidReading(index)) return {CoordType::INVALID, 0.0f};

    float cosA = s.rayCos;
    float sinA = s.raySin;
    float reading = s.readingInch;

    // Compute distances to each of the 4 field walls.  Take the closest
    // positive distance — that's the wall the sensor actually hits first.
    float minDist = 1e9f;
    int wall = -1;  // 1=N(+Y), 2=E(+X), 3=S(-Y), 4=W(-X)

    // X-walls (vertical planes at x = ±FIELD_HALF_WALL)
    if (std::fabs(cosA) > 1e-6f) {
        float dEast = (Field::FIELD_HALF_WALL - s.worldX) / cosA;
        if (dEast > 0.0f && dEast < minDist) { minDist = dEast; wall = 2; }

        float dWest = (-Field::FIELD_HALF_WALL - s.worldX) / cosA;
        if (dWest > 0.0f && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    // Y-walls (horizontal planes at y = ±FIELD_HALF_WALL)
    if (std::fabs(sinA) > 1e-6f) {
        float dNorth = (Field::FIELD_HALF_WALL - s.worldY) / sinA;
        if (dNorth > 0.0f && dNorth < minDist) { minDist = dNorth; wall = 1; }

        float dSouth = (-Field::FIELD_HALF_WALL - s.worldY) / sinA;
        if (dSouth > 0.0f && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    if (wall < 0) return {CoordType::INVALID, 0.0f};  // no wall intersected

    // Derive coordinate from the wall that was hit.
    //   sensor_world = wall_coord - rayComponent * reading
    //   robot = sensor_world - sensorOffsetInWorldFrame
    float result;
    CoordType type;

    if (wall == 1) {       // NORTH — horizontal wall at y = +FIELD_HALF_WALL
        type = CoordType::Y;
        result = Field::FIELD_HALF_WALL - sinA * reading;
    } else if (wall == 2) { // EAST — vertical wall at x = +FIELD_HALF_WALL
        type = CoordType::X;
        result = Field::FIELD_HALF_WALL - cosA * reading;
    } else if (wall == 3) { // SOUTH — horizontal wall at y = -FIELD_HALF_WALL
        type = CoordType::Y;
        result = -Field::FIELD_HALF_WALL - sinA * reading;
    } else {               // wall == 4, WEST — vertical wall at x = -FIELD_HALF_WALL
        type = CoordType::X;
        result = -Field::FIELD_HALF_WALL - cosA * reading;
    }

    // Subtract sensor mount offset from robot center
    float robotHeading = mOdom.getPose().theta;
    float rCos = std::cos(robotHeading);
    float rSin = std::sin(robotHeading);
    float offX = s.mountOffset.x * rCos - s.mountOffset.y * rSin;
    float offY = s.mountOffset.x * rSin + s.mountOffset.y * rCos;

    if (type == CoordType::X) result -= offX;
    else                      result -= offY;

    return {type, result};
}

// ============================================================================
// computePose()
// ============================================================================

std::pair<Pose, int> RclTracking::computePose() const {
    // Independently average X and Y coordinates from different sensors.
    // A sensor pointed at an east/west wall constrains X; a sensor pointed
    // at a north/south wall constrains Y.  The two axes are independent —
    // a sensor that determines X says nothing about Y (and vice versa).
    float sumX = 0.0f, sumY = 0.0f;
    int xCount = 0, yCount = 0;

    for (int i = 0; i < mConfig.sensorCount; i++) {
        auto [type, coord] = getBotCoordFromSensor(i);
        if (type == CoordType::X) {
            sumX += coord;
            xCount++;
        } else if (type == CoordType::Y) {
            sumY += coord;
            yCount++;
        }
    }

    // When no sensor constrains an axis, fall back to odometry
    float estX = (xCount > 0) ? sumX / static_cast<float>(xCount) : mOdom.getPose().x;
    float estY = (yCount > 0) ? sumY / static_cast<float>(yCount) : mOdom.getPose().y;
    int usedCount = xCount + yCount;

    return {
        {estX, estY, mOdom.getPose().theta},
        usedCount
    };
}

// ============================================================================
// update()
// ============================================================================

void RclTracking::update() {
    // Update sensor disable timers
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (mDisableTimers[i] > 0.0f) {
            mDisableTimers[i] -= mConfig.updatePeriodMs;
            if (mDisableTimers[i] <= 0.0f) {
                mSensors[i].enabled = true;
                mDisableTimers[i] = 0.0f;
            }
        }
    }

    Pose currentPose = mOdom.getPose();
    updateSensorPoses(currentPose);

    auto [estimate, sensorCount] = computePose();
    mActiveSensorCount = sensorCount;

    if (sensorCount > 0) {
        if (mConfig.useAccumulation) {
            float alpha = mConfig.accumulationAlpha;
            mLatestEstimate.x = alpha * estimate.x + (1.0f - alpha) * mLatestEstimate.x;
            mLatestEstimate.y = alpha * estimate.y + (1.0f - alpha) * mLatestEstimate.y;
        } else {
            mLatestEstimate = estimate;
        }
        mLatestEstimate.theta = currentPose.theta;
    }

    // Decrement temporary obstacle lifetimes and mark expired ones
    // Permanent obstacles (lifetimeMs == 0) are never removed.
    // Temporary obstacles (lifetimeMs > 0) count down each update;
    // when they cross 0 they are set to -1.0f to flag removal.
    for (auto& o : mLineObstacles) {
        if (o.lifetimeMs > 0.0f) {
            o.lifetimeMs -= mConfig.updatePeriodMs;
            if (o.lifetimeMs <= 0.0f) o.lifetimeMs = -1.0f;
        }
    }
    for (auto& o : mCircleObstacles) {
        if (o.lifetimeMs > 0.0f) {
            o.lifetimeMs -= mConfig.updatePeriodMs;
            if (o.lifetimeMs <= 0.0f) o.lifetimeMs = -1.0f;
        }
    }

    // Remove obstacles flagged for deletion
    mLineObstacles.erase(
        std::remove_if(mLineObstacles.begin(), mLineObstacles.end(),
                       [](const LineObstacle& o) { return o.lifetimeMs < 0.0f; }),
        mLineObstacles.end());
    mCircleObstacles.erase(
        std::remove_if(mCircleObstacles.begin(), mCircleObstacles.end(),
                       [](const CircleObstacle& o) { return o.lifetimeMs < 0.0f; }),
        mCircleObstacles.end());

    if (mConfig.autoSync && sensorCount >= 1) {
        syncUpdate();
    }
}

// ============================================================================
// syncUpdate()
// ============================================================================

void RclTracking::syncUpdate() {
    if (mActiveSensorCount == 0) return;

    Pose odomPose = mOdom.getPose();
    float dx = mLatestEstimate.x - odomPose.x;
    float dy = mLatestEstimate.y - odomPose.y;
    float dist = std::hypot(dx, dy);

    if (dist > mConfig.maxSyncDist) {
        float scale = mConfig.maxSyncDist / dist;
        dx *= scale;
        dy *= scale;
    }

    mOdom.setPose({
        odomPose.x + dx,
        odomPose.y + dy,
        odomPose.theta
    });
}

// ============================================================================
// updateBotPoseFromBestSensor()
// ============================================================================

bool RclTracking::updateBotPoseFromBestSensor() {
    updateSensorPoses(mOdom.getPose());

    int bestIndex = -1;
    float bestDist = 999.0f;

    for (int i = 0; i < mConfig.sensorCount; i++) {
        if (isValidReading(i)) {
            float dist = mSensors[i].readingInch;
            if (dist < bestDist) {
                bestDist = dist;
                bestIndex = i;
            }
        }
    }

    if (bestIndex < 0) return false;

    auto [type, coord] = getBotCoordFromSensor(bestIndex);
    if (type == CoordType::INVALID) return false;

    // Only update the coordinate that this sensor constrains.
    // The other coordinate stays at the odometry value.
    Pose odomPose = mOdom.getPose();
    if (type == CoordType::X) {
        mOdom.setPose({coord, odomPose.y, odomPose.theta});
    } else {
        mOdom.setPose({odomPose.x, coord, odomPose.theta});
    }
    mLatestEstimate = mOdom.getPose();
    return true;
}

// ============================================================================
// Lifecycle
// ============================================================================

void RclTracking::startTracking() {
    if (mTask == nullptr) {
        mRunning = true;
        mLatestEstimate = mOdom.getPose();
        mTask = new pros::Task([this]() { run(); });
    }
}

void RclTracking::stopTracking() {
    mRunning = false;
    if (mTask != nullptr) {
        pros::delay(20);          // let the while(mRunning) loop exit cleanly
        mTask->remove();
        delete mTask;
        mTask = nullptr;
    }
}

bool RclTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void RclTracking::run() {
    while (mRunning) {
        uint32_t startTime = pros::millis();
        // Keep odometry fresh — without this, odom would be stale when
        // the RCL task runs without a concurrent motion command.
        mOdom.update();
        update();
        int32_t elapsed = pros::millis() - startTime;
        int32_t remaining = static_cast<int32_t>(mConfig.updatePeriodMs) - elapsed;
        pros::delay(remaining > 1 ? remaining : 1);
    }
}

}  // namespace FBLIB
