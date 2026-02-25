// Tracking Components
#include "FBLib/Drive/MCL_Tracking.hpp"
#include "FBLib/Drive/Odom_Tracking.hpp"
#include "FBLib/Drive/RCL_Tracking.hpp"
#include "FBLib/Drive/Tracking_Wheels.hpp"

// Movement Control Components
#include "FBLib/Movement_Control/Arc.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace FBLIB {

struct OdomSensors {
  FBLib::TrackingWheel *vert{nullptr};
  FBLib::TrackingWheel *vert2{nullptr};
  FBLib::TrackingWheel *horiz{nullptr};
  FBLib::TrackingWheel *horiz2{nullptr};
  pros::Imu *imu{nullptr};
};

class PIDSettings {
public:
  PIDSettings(float kP, float kI, float kD, float windUpRange, bool flipReset)
      : kP(kP), kI(kI), kD(kD), windUpRange(windUpRange), flipReset(flipReset) {
  }

  float kP;
  float kI;
  float kD;
  float windUpRange;
  bool flipReset;
};

class Drivetrain {
public:
  Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
             float TrackWidth, float WheelDiameter);

  pros::MotorGroup *leftMotors{nullptr};
  pros::MotorGroup *rightMotors{nullptr};
  float TrackWidth;
  float WheelDiameter;
};
enum class TurnDirection {
  CCWCOUNTERCLOCKWISE,
  CWCLOCKWISE,
  SHORTEST
};
struct TurntoHeadingParams {
  TurnDirection direction = TurnDirection::SHORTEST;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches

};
struct TurnToPointParams {
  TurnDirection direction = TurnDirection::SHORTEST;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches

};
struct MoveDistanceParams {
  bool forwards = true;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches

};
struct MoveToPointParams {
  bool forwards = true;
  float maxSpeed = 127;
  float minSpeed = 0;
  float targetTolerance = 1.0; // in inches
};


class Chassis {

public:
  Chassis() {}
};
} // namespace FBLIB