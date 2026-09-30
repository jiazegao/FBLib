#pragma once

#include "FBLib/Movement_Control/Motion.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Parameter structs
// ============================================================================

struct BoomerangParams {
    bool forwards{true};            // false approaches the pose driving backwards
    float maxSpeed{127.0f};
    float minSpeed{0.0f};           // > 0: never slow below this; exits on passing the target (chaining)
    float targetTolerance{1.0f};    // inches
    float headingTolerance{2.0f};   // VEX degrees
    float lead{0.5f};               // carrot offset as a fraction of the distance to the target
    float leadDecay{0.0f};          // fraction of the carrot offset dropped while far from the target
                                    // (0 = constant lead; larger values straighten the approach
                                    // early but leave less room to line up with the final heading)
};

// ============================================================================
// BoomerangMotion — curved approach to a pose
// ============================================================================
//
// Chases a "carrot" point placed behind the target along its final heading
// (in front of it when driving backwards), which bends the path so the
// robot arrives facing the requested direction. Between 12" and
// MOTION_CLOSE_RANGE the steering blends from the carrot to the final
// heading; inside it the robot only holds the final heading and drives on
// the signed distance, so it cannot orbit the target.
// ============================================================================

class BoomerangMotion : public Motion {
public:
    /// @param thetaRad  final heading, standard-math radians
    BoomerangMotion(float x, float y, float thetaRad, const BoomerangParams& params);

    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTargetX;
    float mTargetY;
    float mTargetTheta;
    BoomerangParams mParams;
    bool mClose{false};
};

}  // namespace FBLIB
