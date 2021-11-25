#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>

namespace uav_control
{
    bool fsCheck();
    void fillHeader(std_msgs::Header& header, std::string frameID);
}