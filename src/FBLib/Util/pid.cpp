#include "FBLib/Util/Util.hpp"
#include "FBLib/Util/pid.hpp"
#include "pros/rtos.hpp"

namespace FBLIB {
    // PID
PID::PID(float kP,          // Proportional gain
         float kI,          // Integral gain
         float kD,          // Derivative gain
         float windupRange, // Anti-windup clamp limit
         bool flipReset)    // Negate vs. zero integral on reset
    : mGains{kP, kI, kD}, mWindupRange(windupRange), mFlipReset(flipReset), mPreviousTime(pros::millis()) {}

    PID::PID(const PIDGains& gains, float windupRange, bool flipReset)
        : mGains(gains), mWindupRange(windupRange), mFlipReset(flipReset), mPreviousTime(pros::millis()) {}

    PIDGains PID::getGains() const {
        return mGains;
    }

    void PID::setGains(const PIDGains& gains) {
        mGains = gains;
    }

    float PID::update(float error, float dt) {
        // Clamp dt to prevent integral windup after a long pause between
        // updates (e.g., the first update after construction or calibration).
        if (dt > 0.1f) dt = 0.1f;
        if (dt < 0.0f) dt = 0.0f;

        // proportional term
        float p = mGains.kP * error;

        // integral term with anti-windup
        mIntegral += error * dt;
        if (mWindupRange > 0.0f) {
            if (mIntegral > mWindupRange) {
                mIntegral = mWindupRange;
            } else if (mIntegral < -mWindupRange) {
                mIntegral = -mWindupRange;
            }
        }
        float i = mGains.kI * mIntegral;

        // derivative term with zero-division protection and low pass filtering.
        // The first update after construction/reset has no previous error to
        // difference against — treating it as 0 produced a derivative "kick"
        // of error/dt at the start of every motion.
        float d = 0.0f;
        if (mHasPreviousError && dt > 1e-6f) {
            float derivative = (error - mPreviousError) / dt;
            // Apply low-pass filter to reduce noise amplification
            mFilteredDerivative = mDerivativeFilter * derivative + (1.0f - mDerivativeFilter) * mFilteredDerivative;
            d = mGains.kD * mFilteredDerivative;
        }

        // save error for next derivative calculation
        mPreviousError = error;
        mHasPreviousError = true;

        return p + i + d;
    }

    float PID::update(float error) {
        // Get current time in milliseconds and compute dt
        uint32_t currentTime = pros::millis();
        float dt = (currentTime - mPreviousTime) / 1000.0f;  // Convert to seconds
        mPreviousTime = currentTime;
        return update(error, dt);
    }

    void PID::reset() {
        if (mFlipReset) {
            // Negate the integral so it opposes the opposite direction — this
            // prevents the robot from lurching when the target direction flips.
            mIntegral = -mIntegral;
        } else {
            mIntegral = 0.0f;
        }
        // Forget the previous error — negating it (the old LemLib behavior)
        // causes a derivative kick because the new motion's error has no
        // relationship to the old error's negated magnitude.
        mPreviousError = 0.0f;
        mHasPreviousError = false;
        mFilteredDerivative = 0.0f;
        mPreviousTime = pros::millis();  // Reset time to current
    }

    void PID::setFlipReset(bool flipReset) {
        mFlipReset = flipReset;
    }

    void PID::setWindupRange(float windupRange) {
        mWindupRange = windupRange;
    }

    float PID::getWindupRange() const {
        return mWindupRange;
    }
}
