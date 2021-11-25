#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_msgs/Params.h>
#include <robot_msgs/MissionMap.h>
#include <robot_msgs/RunMission.h>

namespace miss_service_client
{
    class MissServiceClient
    {
        private:
            // ----- ALIASES -----

            using missID = robot_msgs::MissionMap;

            // ----- ROS STUFFS -----

            ros::NodeHandle nh_;
            ros::ServiceClient missClient_;

            // ----- HELPER FUNCTIONS -----
            
            bool clearParams(robot_msgs::Params& params);
            bool fillHeader(std_msgs::Header& header, const std::string& headerID);

        public:
            MissServiceClient();
            ~MissServiceClient() = default;

            bool invokeFunc(const uint8_t& missID, 
                            robot_msgs::Params& params);

    };
}