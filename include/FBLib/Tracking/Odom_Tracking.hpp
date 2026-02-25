#pragma once
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

namespace FBLib {
class TrackingWheel {
public:
  enum class SensorType { ADI, Rotation };

  // ADI encoder constructor
  TrackingWheel(pros::adi::Encoder& enc,
                float wheelDiamIn,
                float offsetIn,
                float gearRatio = 1.0f);

  // Rotation sensor constructor
  TrackingWheel(pros::Rotation& rot,
                float wheelDiamIn,
                float offsetIn,
                float gearRatio = 1.0f);

  // Distance traveled in inches since last reset
  float distanceIn() const;

  // Offset from robot center (inches). Sign convention: +right, -left (you choose and document)
  float offsetIn() const { return mOffsetIn; }

  void reset();

  private:
  SensorType mType;
  pros::adi::Encoder* mAdi{nullptr};
  pros::Rotation* mRot{nullptr};

  float mWheelDiamIn{0};
  float mOffsetIn{0};
  float mGearRatio{1.0f};
};

struct OdomSensors {
  std::vector<TrackingWheel*> vertWheelCollection;
  std::vector<TrackingWheel*> horizWheelCollection;
  std::vector<pros::Imu*> imuCollection;
};

}