#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandHome.h>

#include <geometry_msgs/PoseStamped.h>

#include <geographic_msgs/GeoPointStamped.h>

#include <sensor_msgs/NavSatFix.h>

#include <keyboard/Key.h>

#include <robot_msgs/Params.h>
#include <robot_msgs/MissionMap.h>
#include <robot_msgs/RunMission.h>
#include <robot_msgs/ErrorCodes.h>
#include <robot_msgs/ProcessImage.h>
#include <robot_msgs/ImgProcMap.h>
#include <robot_msgs/GPIO.h>
#include <robot_msgs/GPIOMap.h>
#include <robot_msgs/GeoProcessing.h>
#include <robot_msgs/GeoProcMap.h>

namespace miss_service_server
{
    class MissServiceServer
    {
        private:
            // ----- ALIASES -----

            using Params    = robot_msgs::Params;
            using Errors    = robot_msgs::ErrorCodes;
            using Missions  = robot_msgs::MissionMap;

            // ----- ROS_STUFFS -----

            ros::NodeHandle nh_;
            ros::ServiceServer missSrv_;

            // ----- DATA -----

            std::string mavrosNs_;
            std::string imgProcNs_;
            std::string GPIONs_;

            // ----- CALLBACK FUNCTIONS -----

            bool serviceCB(robot_msgs::RunMission::Request& req, 
                           robot_msgs::RunMission::Response& res);

            // ----- MISSION FUNCTIONS -----

            uint8_t initialize      (const Params& params);
            uint8_t armThrottle     (const Params& params);
            uint8_t disarmThrottle  (const Params& params);
            uint8_t setPosLocal     (const Params& params);
            uint8_t changeMode      (const Params& params);
            uint8_t waitKeyPress    (const Params& params);
            uint8_t saveImg         (const Params& params);
            uint8_t waitTime        (const Params& params);
            uint8_t waitGPS         (const Params& params);
            uint8_t waitForever     (const Params& params);
            uint8_t GPTakeoff       (const Params& params);
            uint8_t takeoffCur      (const Params& params);
            uint8_t GPLand          (const Params& params);
            uint8_t landCur         (const Params& params);
            uint8_t geotag          (const Params& params);
            uint8_t waitWP          (const Params& params);
            uint8_t waitCamTrig     (const Params& params);
            uint8_t waitSaveImg     (const Params& params);

        public:
            MissServiceServer();
            ~MissServiceServer() = default;
    };
}
