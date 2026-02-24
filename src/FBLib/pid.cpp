#include "FBLib/pid.hpp"

namespace FBLib {
    // PID 
PID::PID(float kP,          // Proportional gain
         float kI,          // Integral gain
         float kD,          // Derivative gain
         float windupRange, // Anti-windup clamp limit
         bool flipReset)    // Negate vs. zero integral on reset
    : mGains{kP, kI, kD}, mWindupRange(windupRange), mFlipReset(flipReset) {}

    PID::PID(const PIDGains& gains, float windupRange, bool flipReset)
        : mGains(gains), mWindupRange(windupRange), mFlipReset(flipReset) {}

    PIDGains PID::getGains() const {
        return mGains;
    }

    void PID::setGains(const PIDGains& gains) {
        mGains = gains;
    }

    float PID::update(float error, float dt) {
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

        // derivative term
        float derivative = (error - mPreviousError) / dt;
        float d = mGains.kD * derivative;

        // save error for next derivative calculation
        mPreviousError = error;

        return p + i + d;
    }

    void PID::reset() {
        if (mFlipReset) {
            mIntegral = -mIntegral;
            mPreviousError = -mPreviousError;
        } else {
            mIntegral = 0.0f;
            mPreviousError = 0.0f;
        }
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