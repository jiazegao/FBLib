#pragma once

namespace FBLib {

    struct PIDGains {
        float kP = 0.0f;
        float kI = 0.0f;
        float kD = 0.0f;
    };

    class PID {
    public:
        PID(float kP, float kI, float kD, float windupRange = 0.0f, bool flipReset = false);
        PID(const PIDGains& gains, float windupRange = 0.0f, bool flipReset = false);

        PIDGains getGains() const;
        void setGains(const PIDGains& gains);

        float update(float error, float dt);
        void reset();

        void setFlipReset(bool flipReset);
        void setWindupRange(float windupRange);
        float getWindupRange() const;

    private:
        PIDGains mGains;
        float mWindupRange;
        bool mFlipReset;
        float mIntegral = 0.0f;
        float mPreviousError = 0.0f;
    };
}
