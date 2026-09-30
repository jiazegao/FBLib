#pragma once

#include <vector>

#include "FBLib/Movement_Control/Motion.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// RAMSETE utility functions
// ============================================================================

/// Find the closest point index on a path to a given pose
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose);

/// Find the lookahead point index on a path (first point beyond lookaheadDist)
int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist);

// ============================================================================
// Parameter structs
// ============================================================================

struct RAMSETEParams {
    // b and zeta use the conventional (meter-based) RAMSETE parameterization —
    // the controller scales b to inches internally, so the textbook defaults
    // b≈2.0, zeta∈(0,1) behave sensibly without retuning.
    float b{2.0f};       // aggressiveness (higher = tighter tracking), b > 0
    float zeta{0.7f};    // damping ratio, 0 < zeta < 1
    float maxSpeed{127.0f};            // peak output, motor units (0–127)
    float minSpeed{0.0f};              // floor on the profiled speed, motor units (path entry/exit speed)
    float targetTolerance{1.0f};       // inches
    float headingTolerance{2.0f};      // VEX degrees
    float lookaheadDist{2.0f};         // reference point this far ahead along the path (inches);
                                       // long lookaheads cut corners and override the speed profile
    bool useVelocityProfile{true};     // profile v_d for accel/decel along path
    float maxAccel{50.0f};             // in/s², used when useVelocityProfile
};

// ============================================================================
// RamseteMotion — nonlinear SE(2) trajectory tracking along a pose path
// ============================================================================
//
// Path poses need meaningful theta (standard-math radians, the direction of
// travel). The reference is the path point `lookaheadDist` inches of arc
// length ahead of the robot's progress, which only ever moves forward. The
// motion ends within tolerance of the final pose, or once the robot crosses
// the line through the endpoint perpendicular to the path.
// ============================================================================

class RamseteMotion : public Motion {
public:
    RamseteMotion(const std::vector<Pose>& path, const RAMSETEParams& params);

    void start(const Pose& pose, const MotionContext& ctx) override;
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    /// Arc length from path[0] to the robot's projection near path[mClosest].
    float progressAlongPath(const Pose& pose) const;

    std::vector<Pose> mPath;
    RAMSETEParams mParams;
    std::vector<float> mCumLen;   // arc length from path[0] to path[i]
    float mTotalLen{0.0f};
    float mCruise{0.0f};          // peak speed, in/s
    float mEndTangent{0.0f};      // direction of travel at the endpoint
    int mClosest{0};
};

}  // namespace FBLIB
