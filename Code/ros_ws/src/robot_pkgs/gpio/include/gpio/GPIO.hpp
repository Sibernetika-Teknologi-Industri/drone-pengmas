#pragma once

#include <ros/ros.h>

#include <wiringPi.h>

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>

#include <robot_msgs/ErrorCodes.h>
#include <robot_msgs/Params.h>
#include <robot_msgs/GPIO.h>
#include <robot_msgs/GPIOMap.h>

namespace gpio
{
    class GPIO
    {
        private:
            // ----- ALIASES -----

            using Params    = robot_msgs::Params;
            using Errors    = robot_msgs::ErrorCodes;
            using Funcs     = robot_msgs::GPIOMap;

            // ----- ROS_STUFFS -----

            ros::NodeHandle nh_;
            ros::ServiceServer GPIOSrv_;

            // ----- DATA -----

            std::string mavrosNs_;
            int camTrigPin_;
            int camFeedPin_;
            int CAM_DURATION_;
            int CAM_FEEDBACK_POL_;
            int CAM_RELAY_ON_;

            // ----- CALLBACKS -----

            bool serviceCB(robot_msgs::GPIO::Request& req, 
                           robot_msgs::GPIO::Response& res);

            // ----- GPIO FUNCTIONS -----

            uint8_t waitCamTrig (const Params& params);
            uint8_t camFeedback (const Params& params);

        public:
            GPIO();
            ~GPIO() = default;
    };
}
