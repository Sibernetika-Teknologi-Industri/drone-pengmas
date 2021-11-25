/**
 * This file is the main file for running this package
 */

#include <uav_control/uav_control_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "uav_control_node");

    // Multithreading
    constexpr u_int8_t threads{2}; // number of threads
    ros::AsyncSpinner spinner(2);

    

    return 0;
}