/**
 * This file is for launching mission functions server node
 */

#include <mission_control/miss_server_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mission_server");

    // Initialize mission functions server
    miss_service_server::MissServiceServer funcsServer;

    // Run node
    ros::spin();

    return 0;
}