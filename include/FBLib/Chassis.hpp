// Tracking Components
#include "FBLib/Tracking/MCL_Tracking.hpp"
#include "FBLib/Tracking/Odom_Tracking.hpp"
#include "FBLib/Tracking/RCL_Tracking.hpp"

// Movement Control Components
#include "FBLib/Movement_Control/Arc.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace FBLIB {

class Drivetrain {
public:
  Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
             float TrackWidth, float WheelDiameter);

  pros::MotorGroup *leftMotors{nullptr};
  pros::MotorGroup *rightMotors{nullptr};
  float TrackWidth;
  float WheelDiameter;
};

class Chassis {

public:
  Chassis() {}
};

}