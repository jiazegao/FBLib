#include "main.h"

// Tracking Components
#include "FBLib/Chassis/MCL_Tracking.hpp"
#include "FBLib/Chassis/RCL_Tracking.hpp"
#include "FBLib/Chassis/Odom_Tracking.hpp"
#include "FBLib/Chassis/Tracking_Wheels.hpp"
// Movement Control Components
#include "FBLib/Movement_Control/Arc.hpp"
#include "pros/imu.hpp"


namespace FBLIB {

    struct OdomSensors {

        FBLib::TrackingWheel* vert{nullptr};
        FBLib::TrackingWheel* vert2{nullptr};
        FBLib::TrackingWheel* horiz{nullptr};
        FBLib::TrackingWheel* horiz2{nullptr};
        pros::Imu* imu{nullptr};
    };

    class PIDSettings {

        public:

            PIDSettings() {

            }

    };

    class Drivetrain {

        public:

            Drivetrain() {

            }
    };






    class Chassis {

        public:

            Chassis() {

            }

    };
}