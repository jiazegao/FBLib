#include "FBLib/Movement_Control/Velocity_Profiles.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

namespace {

/// Upper bound on samples per profile so degenerate inputs (e.g. a tiny
/// maxVel) cannot allocate unbounded memory on the brain.
constexpr std::size_t MAX_PROFILE_SAMPLES = 20000;

/// Number of dt steps covering [0, tTotal], widening dt if it would exceed
/// MAX_PROFILE_SAMPLES.
std::size_t sampleSteps(float tTotal, float& dt) {
    float steps = std::ceil(tTotal / dt);
    if (steps > static_cast<float>(MAX_PROFILE_SAMPLES)) {
        dt = tTotal / static_cast<float>(MAX_PROFILE_SAMPLES);
        steps = static_cast<float>(MAX_PROFILE_SAMPLES);
    }
    return static_cast<std::size_t>(std::max(steps, 1.0f));
}

}  // namespace

std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt) {
    std::vector<ProfilePoint> profile;
    if (!(distance > 0.0f) || !(maxVel > 0.0f) || !(maxAccel > 0.0f) || !(dt > 0.0f)) {
        profile.push_back({0.0f, 0.0f, 0.0f});
        return profile;
    }

    // Time to accelerate to max velocity
    float tAccel = maxVel / maxAccel;
    // Distance covered during acceleration
    float dAccel = 0.5f * maxAccel * tAccel * tAccel;

    // Check if we even reach max velocity (triangular profile)
    float tCruise, dCruise;
    if (2.0f * dAccel > distance) {
        // Triangular: never reaches maxVel. Half the distance accelerating,
        // half decelerating (dAccel must be recomputed, or the deceleration
        // phase starts from the full-speed ramp's distance).
        tAccel = std::sqrt(distance / maxAccel);
        maxVel = maxAccel * tAccel;
        dAccel = 0.5f * distance;
        tCruise = 0.0f;
        dCruise = 0.0f;
    } else {
        dCruise = distance - 2.0f * dAccel;
        tCruise = dCruise / maxVel;
    }

    float tTotal = 2.0f * tAccel + tCruise;
    std::size_t steps = sampleSteps(tTotal, dt);
    profile.reserve(steps + 1);

    for (std::size_t k = 0; k <= steps; k++) {
        float t = static_cast<float>(k) * dt;
        float vel, pos;

        if (t < tAccel) {
            // Accelerating
            vel = maxAccel * t;
            pos = 0.5f * maxAccel * t * t;
        } else if (t < tAccel + tCruise) {
            // Cruising
            vel = maxVel;
            pos = dAccel + maxVel * (t - tAccel);
        } else if (t < tTotal) {
            // Decelerating
            float tDecel = t - tAccel - tCruise;
            vel = maxVel - maxAccel * tDecel;
            pos = dAccel + dCruise + maxVel * tDecel - 0.5f * maxAccel * tDecel * tDecel;
        } else {
            // Done
            vel = 0.0f;
            pos = distance;
        }

        profile.push_back({t, vel, clamp(pos, 0.0f, distance)});
    }

    return profile;
}

std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                          float maxAccel, float maxJerk,
                                          float dt) {
    if (!(distance > 0.0f) || !(maxVel > 0.0f) || !(maxAccel > 0.0f) || !(dt > 0.0f)) {
        return {{0.0f, 0.0f, 0.0f}};
    }
    if (!(maxJerk > 0.0f)) {
        return generateTrapezoidal(distance, maxVel, maxAccel, dt);  // unlimited jerk
    }

    // ========================================================================
    // 7-phase S-curve:
    //   1. Jerk up   (accel: 0 → +a)       4. Cruise    (constant v)
    //   2. Const accel                      5-7. Mirror of 1-3 (deceleration)
    //   3. Jerk down (accel: +a → 0)
    //
    // Reaching speed v from rest:
    //   v >= A²/J : accel saturates at A:  tJ = A/J, tA = v/A − A/J
    //   v <  A²/J : accel peaks at a = √(vJ) < A:  tJ = √(v/J), tA = 0
    // The velocity curve of the ramp is point-symmetric, so it covers
    // v·(2·tJ + tA)/2. Moves too short for maxVel lower the peak speed
    // (bisection — ramp distance grows monotonically with v).
    // ========================================================================

    const float A = maxAccel;
    const float J = maxJerk;

    struct Ramp {
        float tJ, tA, a;  // jerk-phase time, constant-accel time, peak accel
    };
    auto rampFor = [A, J](float v) {
        Ramp r;
        if (v >= A * A / J) {
            r.a = A;
            r.tJ = A / J;
            r.tA = v / A - A / J;
        } else {
            r.a = std::sqrt(v * J);
            r.tJ = r.a / J;
            r.tA = 0.0f;
        }
        return r;
    };
    auto rampDistance = [&rampFor](float v) {
        Ramp r = rampFor(v);
        return 0.5f * v * (2.0f * r.tJ + r.tA);
    };

    float vPeak = maxVel;
    if (2.0f * rampDistance(maxVel) > distance) {
        float lo = 0.0f, hi = maxVel;
        for (int i = 0; i < 60; i++) {
            float mid = 0.5f * (lo + hi);
            if (2.0f * rampDistance(mid) > distance) hi = mid;
            else lo = mid;
        }
        vPeak = lo;
    }

    const Ramp ramp = rampFor(vPeak);
    const float tRamp = 2.0f * ramp.tJ + ramp.tA;
    const float dRamp = 0.5f * vPeak * tRamp;
    const float dCruise = std::max(distance - 2.0f * dRamp, 0.0f);
    const float tCruise = (vPeak > 0.0f) ? dCruise / vPeak : 0.0f;
    const float tTotal = 2.0f * tRamp + tCruise;

    // Velocity/position during the acceleration ramp, t in [0, tRamp]
    const float v1 = 0.5f * J * ramp.tJ * ramp.tJ;
    const float s1 = J * ramp.tJ * ramp.tJ * ramp.tJ / 6.0f;
    const float v2 = v1 + ramp.a * ramp.tA;
    const float s2 = s1 + v1 * ramp.tA + 0.5f * ramp.a * ramp.tA * ramp.tA;
    auto rampState = [&](float t, float& v, float& s) {
        if (t < ramp.tJ) {                              // phase 1: a = J·t
            v = 0.5f * J * t * t;
            s = J * t * t * t / 6.0f;
        } else if (t < ramp.tJ + ramp.tA) {             // phase 2: a = peak
            float tau = t - ramp.tJ;
            v = v1 + ramp.a * tau;
            s = s1 + v1 * tau + 0.5f * ramp.a * tau * tau;
        } else {                                        // phase 3: a = peak − J·τ
            float tau = std::min(t - ramp.tJ - ramp.tA, ramp.tJ);
            v = v2 + ramp.a * tau - 0.5f * J * tau * tau;
            s = s2 + v2 * tau + 0.5f * ramp.a * tau * tau - J * tau * tau * tau / 6.0f;
        }
    };

    std::vector<ProfilePoint> profile;
    std::size_t steps = sampleSteps(tTotal, dt);
    profile.reserve(steps + 1);
    for (std::size_t k = 0; k <= steps; k++) {
        float t = std::min(static_cast<float>(k) * dt, tTotal);
        float v, s;
        if (t <= tRamp) {
            rampState(t, v, s);
        } else if (t <= tRamp + tCruise) {
            v = vPeak;
            s = dRamp + vPeak * (t - tRamp);
        } else {
            // Deceleration mirrors the ramp in time
            rampState(tTotal - t, v, s);
            s = distance - s;
        }
        profile.push_back({t, std::max(v, 0.0f), clamp(s, 0.0f, distance)});
    }
    profile.back() = {tTotal, 0.0f, distance};
    return profile;
}

float trapezoidalVelocityAt(float position, float distance, float maxVel,
                            float maxAccel, float edgeVel) {
    if (!(maxVel > 0.0f)) return 0.0f;
    if (!(maxAccel > 0.0f)) return maxVel;
    float length = std::max(distance, 0.0f);
    float s = clamp(position, 0.0f, length);
    float v0 = clamp(edgeVel, 0.0f, maxVel);
    float accelLimited = std::sqrt(v0 * v0 + 2.0f * maxAccel * s);
    float decelLimited = std::sqrt(v0 * v0 + 2.0f * maxAccel * (length - s));
    return std::min(maxVel, std::min(accelLimited, decelLimited));
}

}  // namespace FBLIB
