/**
 * This file is for utilies and helper functions
 */

#include <uav_control/utils.hpp>

namespace uav_control
{
    // To check if failsafe is triggered or not
    bool fsCheck()
    {

        return false;
    }

    // To fill header
    void fillHeader(std_msgs::Header& header, std::string frameID)
    {
        header.stamp = ros::Time::now();
    }
}
