#pragma once

#include "FBLib/Movement_Control/Motion.hpp"
#include "FBLib/Util/Util.hpp"

namespace FBLIB {

// ============================================================================
// Parameter structs
// ============================================================================

struct MoveDistanceParams {
    bool forwards{true};            // false drives the distance backwards
    float maxSpeed{127.0f};
    float minSpeed{0.0f};           // > 0: never slow below this; exits on passing the target (chaining)
    float targetTolerance{1.0f};    // inches
    float earlyExitRange{0.0f};     // exit early if within this range (0 = disabled)
};

struct MoveToPointParams {
    bool forwards{true};            // false approaches the point driving backwards
    float maxSpeed{127.0f};
    float minSpeed{0.0f};           // > 0: never slow below this; exits on passing the target (chaining)
    float targetTolerance{1.0f};    // inches
    float earlyExitRange{0.0f};     // exit early if within this range (0 = disabled)
    float lead{0.0f};               // no effect: a point has no final heading to curve into.
                                    // Use moveBoomerang() for a curved approach.
};

// ============================================================================
// MoveDistanceMotion — drive a signed distance, holding the start heading
// ============================================================================
//
// The target is fixed once at start(): `distance` inches along the heading
// the robot has at that moment (negative distance, or forwards=false, drives
// backwards). Progress is measured as the signed projection onto that
// heading, so the robot never turns around to chase the target point.
// ============================================================================

class MoveDistanceMotion : public Motion {
public:
    MoveDistanceMotion(float distance, const MoveDistanceParams& params);

    void start(const Pose& pose, const MotionContext& ctx) override;
    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mDistance;
    MoveDistanceParams mParams;
    float mHeading{0.0f};     // heading held for the whole motion (rad)
    float mTargetX{0.0f};
    float mTargetY{0.0f};
    float mTravelSign{1.0f};  // +1 driving forwards, -1 backwards
};

// ============================================================================
// MoveToPointMotion — drive to a field coordinate
// ============================================================================
//
// Steers at the target while far away and drives on the signed projection
// of the error (negative when the target is behind). Inside
// MOTION_CLOSE_RANGE it first turns in place until it is aimed at the
// target, then stops steering and drives straight, so overshooting backs the
// robot up instead of spinning it around the point.
// ============================================================================

class MoveToPointMotion : public Motion {
public:
    MoveToPointMotion(float x, float y, const MoveToPointParams& params);

    MotionOutput update(const Pose& pose, float dt, const MotionContext& ctx) override;

private:
    float mTargetX;
    float mTargetY;
    MoveToPointParams mParams;
    bool mLocked{false};  // aimed at the target inside MOTION_CLOSE_RANGE: heading held
};

}  // namespace FBLIB
