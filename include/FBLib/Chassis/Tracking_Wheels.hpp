#pragma once

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

namespace FBLib {
    /**< TrackingWheel class for tracking wheels > */
    
    class TrackingWheel {
        public:
            pros::adi::Encoder adi_encoder;
            pros::Rotation rotsensor;
        
        /*  create a constructor for the 3-wire shaft encoder
        @param WheelDiameter: Diameter of the tracking wheel in inches
        @param distance: Distance from the center of the tracking wheel to the center of the robot in inches
        @param gearRatio: Gear ratio between the tracking wheel and the encoder (default is 1, meaning 1:1)       
        *
        */
        TrackingWheel(pros::adi::Encoder* encoder, float WheelDiameter, float distance, float gearRatio = 1);
        /*create a constructor for the V5 rotation sensor
        @param WheelDiameter: Diameter of the tracking wheel in inches
        @param distance: Distance from the center of the tracking wheel to the center of the robot
        @param gearRatio: Gear ratio between the tracking wheel and the encoder (default is 1, meaning 1:1)
        */
        TrackingWheel(pros::Rotation* encoder, float WheelDiameter, float distance, float gearRatio = 1);        
        // add no tracker odom here:
/*------------------------------------------*/
        /**
        * Returns the distance traveled by the tracking wheel in inches. This is calculated using the number of rotations of the wheel, the diameter of the wheel, and the gear ratio.
        */
        float getDistance(); 
        /** 
        *Get the offset of the tracking wheel from the center of the robot in inches
         */
        float Offset();
        /*
        * Resets the value of the tracking wheel to zero. This is useful for resetting the tracking wheel at the start of a match or after a certain event.
        */
        void reset();
        /*
        *Returns the current RPM of the tracking wheel. This can be used for velocity control or to monitor the speed of the robot.
        */
        float getDistanceTraveled();
    private:
        float mWheelDiameter;
        float mDistanceFromCenter;
        float rpm;
        pros::adi::Encoder* AdiEncoder{nullptr};
        pros::Rotation* Rotsensor{nullptr};
        float mGearRatio = 1;
};
} //namespace FBLib