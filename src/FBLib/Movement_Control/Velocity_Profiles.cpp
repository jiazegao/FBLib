#include "FBLib/Movement_Control/Velocity_Profiles.hpp"

#include <algorithm>
#include <cmath>

#include "FBLib/Util/Util.hpp"

namespace FBLIB {

std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt) {
    std::vector<ProfilePoint> profile;

    // Time to accelerate to max velocity
    float tAccel = maxVel / maxAccel;
    // Distance covered during acceleration
    float dAccel = 0.5f * maxAccel * tAccel * tAccel;

    // Check if we even reach max velocity (triangular profile)
    float tCruise, dCruise;
    if (2.0f * dAccel > distance) {
        // Triangular: never reaches maxVel
        tAccel = std::sqrt(distance / maxAccel);
        maxVel = maxAccel * tAccel;
        tCruise = 0.0f;
        dCruise = 0.0f;
    } else {
        dCruise = distance - 2.0f * dAccel;
        tCruise = dCruise / maxVel;
    }

    float tTotal = 2.0f * tAccel + tCruise;
    float pos = 0.0f;

    for (float t = 0.0f; t <= tTotal + dt; t += dt) {
        float vel, accel;

        if (t < tAccel) {
            // Accelerating
            accel = maxAccel;
            vel = maxAccel * t;
            pos = 0.5f * maxAccel * t * t;
        } else if (t < tAccel + tCruise) {
            // Cruising
            accel = 0.0f;
            vel = maxVel;
            pos = dAccel + maxVel * (t - tAccel);
        } else if (t < tTotal) {
            // Decelerating
            float tDecel = t - tAccel - tCruise;
            accel = -maxAccel;
            vel = maxVel - maxAccel * tDecel;
            pos = dAccel + dCruise + maxVel * tDecel - 0.5f * maxAccel * tDecel * tDecel;
        } else {
            // Done
            accel = 0.0f;
            vel = 0.0f;
            pos = distance;
        }

        profile.push_back({t, vel, pos});
    }

    return profile;
}

std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                          float maxAccel, float maxJerk,
                                          float dt) {
    std::vector<ProfilePoint> profile;

    // Guard: zero or negative distance
    if (distance <= 0.0f) {
        profile.push_back({0.0f, 0.0f, 0.0f});
        return profile;
    }

    // ========================================================================
    // Compute phase durations for a proper 7-phase S-curve:
    //   1. Jerk up   (accel: 0 → +A)
    //   2. Const accel
    //   3. Jerk down (accel: +A → 0, speed peaks)
    //   4. Cruise    (constant v)
    //   5. Jerk down (accel: 0 → -A)
    //   6. Const decel
    //   7. Jerk up   (accel: -A → 0)
    //
    // Phases 1,3,5,7 each have duration tJ = A / J.
    // Phases 2,6 each have duration tA (may be zero).
    // Phase 4 has duration tC (may be zero).
    // ========================================================================

    // Time to reach max accel from zero at max jerk
    float tJ = maxAccel / maxJerk;

    // Velocity gained in a single jerk-only ramp (no constant-accel phase):
    //   v(t) = J·t²/2  at t = tJ → vJ = A² / (2·J)
    float vJ = 0.5f * maxAccel * maxAccel / maxJerk;

    // Distance covered in a single jerk-only ramp:
    //   s(t) = J·t³/6  at t = tJ → sJ = A³ / (6·J²)
    float sJ = maxAccel * maxAccel * maxAccel / (6.0f * maxJerk * maxJerk);

    // Can we reach maxVel with full acceleration?
    // Velocity after phases 1+2+3 = 2·vJ + A·tA  where tA = constant phase
    float tA = (maxVel - 2.0f * vJ) / maxAccel;
    bool fullAccel = (tA >= 0.0f);

    float vPeak, aPeak;
    if (fullAccel) {
        vPeak = maxVel;
        aPeak = maxAccel;
    } else {
        // Cannot reach maxVel — triangular acceleration (no constant-accel phase).
        // Use the actual maxAccel; peak velocity is limited by accel + jerk alone.
        tA = 0.0f;
        aPeak = maxAccel;
        vPeak = 2.0f * vJ;   // max speed reachable with jerk-only ramps
    }

    // — Distance during a full acceleration half (phases 1→3) —
    // Phase 1: s1 = sJ,  v1 = vJ
    // Phase 2: s2 = s1 + v1·tA + ½·A·tA²,  v2 = v1 + A·tA
    // Phase 3: s3 = s2 + v2·tJ + ½·A·tJ² − J·tJ³/6 = s2 + v2·tJ + ½·A·tJ² − sJ
    //   v3 = v2 + A·tJ − ½·J·tJ² = v2 + vJ

    float a = aPeak;
    float v1 = vJ;
    float s1 = sJ;
    float v2 = v1 + a * tA;
    float s2 = s1 + v1 * tA + 0.5f * a * tA * tA;
    float v3 = v2 + vJ;                               // = vPeak
    float s3 = s2 + v2 * tJ + 0.5f * a * tJ * tJ - sJ;

    float dAccel = s3;     // distance during acceleration half
    float dDecel = dAccel; // symmetric deceleration
    float dCruise = distance - dAccel - dDecel;

    // — If distance is too short, reduce vPeak to fit —
    if (dCruise < 0.0f) {
        if (distance < 1.0f) {
            return generateTrapezoidal(distance, maxVel, maxAccel, dt);
        }

        // Solve for vPeak that makes dCruise ≈ 0.
        // The accel distance scales roughly as: dAccel ≈ (vPeak/A)² · A³/(3·J²)
        // for jerk-only, or with a constant-accel term otherwise.
        // Iterate to converge (usually 3-4 iterations).
        float vTarget = vPeak;
        for (int iter = 0; iter < 8; iter++) {
            tA = std::max(0.0f, (vTarget - 2.0f * vJ) / a);
            float vt1 = vJ;
            float st1 = sJ;
            float vt2 = vt1 + a * tA;
            float st2 = st1 + vt1 * tA + 0.5f * a * tA * tA;
            float st3 = st2 + vt2 * tJ + 0.5f * a * tJ * tJ - sJ;
            float dTotal = 2.0f * st3;
            if (dTotal <= distance + 0.001f) break;
            vTarget -= (dTotal - distance) / (2.0f * tJ + tA + 0.01f); // approximate derivative
            if (vTarget < 0.01f) vTarget = 0.01f;
        }
        vPeak = vTarget;
        tA = std::max(0.0f, (vPeak - 2.0f * vJ) / a);
        // Recompute with converged vPeak
        v2 = vJ + a * tA;
        s2 = sJ + vJ * tA + 0.5f * a * tA * tA;
        v3 = v2 + vJ;
        s3 = s2 + v2 * tJ + 0.5f * a * tJ * tJ - sJ;
        dAccel = s3;
        dCruise = 0.0f;
    }

    float tC = dCruise / vPeak;  // cruise duration

    // — Phase time boundaries —
    float T1 = tJ;
    float T2 = T1 + tA;
    float T3 = T2 + tJ;           // end of accel
    float T4 = T3 + tC;           // end of cruise
    float T5 = T4 + tJ;           // end of decel jerk-in
    float T6 = T5 + tA;           // end of const decel
    float T7 = T6 + tJ;           // end of decel

    // ========================================================================
    // Sample the profile at dt intervals
    // ========================================================================

    for (float t = 0.0f; t <= T7 + dt; t += dt) {
        float pos_t, vel_t;

        if (t <= 0.0f) {
            vel_t = 0.0f; pos_t = 0.0f;
        } else if (t < T1) {
            // Phase 1: jerk up — a(τ) = +J·τ
            float tau = t;
            vel_t = 0.5f * maxJerk * tau * tau;
            pos_t = maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T2) {
            // Phase 2: constant +accel — a(τ) = +A
            float tau = t - T1;
            vel_t = v1 + a * tau;
            pos_t = s1 + v1 * tau + 0.5f * a * tau * tau;
        } else if (t < T3) {
            // Phase 3: jerk down — a(τ) = +A − J·τ
            float tau = t - T2;
            vel_t = v2 + a * tau - 0.5f * maxJerk * tau * tau;
            pos_t = s2 + v2 * tau + 0.5f * a * tau * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T4) {
            // Phase 4: cruise — a = 0, v = vPeak
            float tau = t - T3;
            vel_t = vPeak;
            pos_t = s3 + vPeak * tau;
        } else if (t < T5) {
            // Phase 5: jerk into decel — a(τ) = −J·τ
            float tau = t - T4;
            vel_t = vPeak - 0.5f * maxJerk * tau * tau;
            pos_t = s3 + dCruise + vPeak * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T6) {
            // Phase 6: constant −accel — a(τ) = −A
            float tau = t - T5;
            float v5 = vPeak - vJ;                                  // v at start of phase 6
            float s5 = s3 + dCruise + vPeak * tJ - sJ;              // s at start of phase 6
            vel_t = v5 - a * tau;
            pos_t = s5 + v5 * tau - 0.5f * a * tau * tau;
        } else if (t < T7) {
            // Phase 7: jerk to zero — a(τ) = −A + J·τ
            float tau = t - T6;
            float v6 = vJ;                                           // v at start of phase 7
            float s6 = distance - dAccel;                            // s at start of phase 7 (symmetric)
            vel_t = v6 - a * tau + 0.5f * maxJerk * tau * tau;
            pos_t = s6 + v6 * tau - 0.5f * a * tau * tau
                  + maxJerk * tau * tau * tau / 6.0f;
        } else {
            vel_t = 0.0f;
            pos_t = distance;
        }

        pos_t = clamp(pos_t, 0.0f, distance);
        profile.push_back({t, vel_t, pos_t});

        if (pos_t >= distance - 0.001f && std::fabs(vel_t) < 0.01f) break;
    }

    // Ensure final point
    if (profile.empty() || profile.back().position < distance - 0.01f) {
        float tFinal = profile.empty() ? 0.0f : profile.back().time + dt;
        profile.push_back({tFinal, 0.0f, distance});
    }

    return profile;
}

}  // namespace FBLIB
