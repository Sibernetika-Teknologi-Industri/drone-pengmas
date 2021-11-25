/**
 * This file is for launching mission functions client node
 */

#include <mission_control/miss_client_node.hpp>

int main(int argc, char* argv[])
{
    // Aliases
    using missID = robot_msgs::MissionMap;

    ros::init(argc, argv, "mission_client");

    robot_msgs::Params params;

    // Initialize mission functions client
    miss_service_client::MissServiceClient funcsClient;

    ROS_INFO("Initializing");

    // Initialize robot
    //funcsClient.invokeFunc(missID::INITIALIZE, params);

    // Change mode
    /*params.strings.push_back("GUIDED");
    funcsClient.invokeFunc(missID::CHANGE_MODE, params);*/

    //ROS_INFO("Waiting for q to be pressed");

    // Wait for key press
    /*params.uints.push_back(keyboard::Key::KEY_q);
    params.floats.push_back(10.0);
    funcsClient.invokeFunc(missID::WAIT_KEY_PRESS, params);*/

    //ROS_INFO("Mission starting");

    // Arm throttle
    //funcsClient.invokeFunc(missID::ARM_THROTTLE, params);

    // Takeoff
    /*params.floats.push_back(5.0f);
    funcsClient.invokeFunc(missID::TAKEOFF_CUR, params);*/

    // Wait for one second
    /*params.floats.push_back(1.0f);
    params.floats.push_back(0.0f);
    funcsClient.invokeFunc(missID::WAIT_TIME, params);*/
    
    // Save image
    /*params.strings.push_back("test");
    params.strings.push_back("test.png");
    funcsClient.invokeFunc(missID::SAVE_IMG, params);*/

    // Geotag
    /*params.strings.push_back("test");
    params.strings.push_back("test.png");
    params.strings.push_back("test.csv");
    funcsClient.invokeFunc(missID::GEOTAG, params);*/

    // Wait for one second
    /*params.floats.push_back(1.0f);
    params.floats.push_back(0.0f);
    funcsClient.invokeFunc(missID::WAIT_TIME, params);*/

    // Land
    //funcsClient.invokeFunc(missID::LAND_CUR, params);

    // Disarm throttle
    //funcsClient.invokeFunc(missID::DISARM_THROTTLE, params);

    // Wait for camera trigger command is sent
    params.floats.push_back(0);
    funcsClient.invokeFunc(missID::WAIT_CAM_TRIG, params);

    ROS_INFO("Mission finished");

    // Run node
    ros::spin();

    return 0;
}