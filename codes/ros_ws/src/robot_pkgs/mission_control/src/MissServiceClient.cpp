/**
 * This file is for mission functions client
 * Implemented using service architecture
 */

#include <mission_control/MissServiceClient.hpp>

namespace miss_service_client
{
    MissServiceClient::MissServiceClient() :
        missClient_(nh_.serviceClient<robot_msgs::RunMission>("mission_server")) // for service call to mission server
    {
        ROS_INFO("Mission client initializing");

        // Check and set log verbosity level
        bool showDbgMsg{false};
        if (nh_.getParam("/show_dbg_msg", showDbgMsg))
        {
            if (showDbgMsg)
            {
                if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
                {
                    ros::console::notifyLoggerLevelsChanged();
                }   
            }

            ROS_INFO("Log verbosity level is set to DEBUG");
        }

        // Wait until service exists
        ROS_DEBUG("Waiting for mission server");
        ROS_DEBUG_STREAM("Client node: " << ros::this_node::getName());
        missClient_.waitForExistence();
        ROS_DEBUG("Mission server exists, waiting stopped");

        ROS_INFO_STREAM("Mission client initialized at " << ros::this_node::getName());
    }

    // ----- PRIVATE METHODS -----

    // ----- HELPER FUNCTIONS -----

    /**
     * Function to clear params
     * Returns true if succesfull, and otherwise
     */
    bool MissServiceClient::clearParams(robot_msgs::Params& params)
    {
        params.floats.clear();
        params.ints.clear();
        params.strings.clear();
        params.uints.clear();

        return true;
    }

    /**
     * Function to fill header
     * Returns true if succesfull, and otherwise
     */
    bool MissServiceClient::fillHeader(std_msgs::Header& header, const std::string& headerID)
    {
        header.frame_id = headerID;
        header.stamp = ros::Time::now();

        return true;
    }

    // ----- PUBLIC METHODS -----

    /**
     * Function to invoke mission functions
     * Will return true if service is succesfully called, and otherwise
     * Will automatically fill header time stamp and frame ID
     * Will also automatically clear mission params
     */ 
    bool MissServiceClient::invokeFunc(const uint8_t& missID, 
                                       robot_msgs::Params& params)
    {
        std::string funcName;
        switch (missID)
        {
            case missID::ARM_THROTTLE:
                funcName = "armThrottle";
                break;
            
            case missID::CHANGE_MODE:
                funcName = "changeMode";
                break;

            case missID::DISARM_THROTTLE:
                funcName = "disarmThrottle";
                break;

            case missID::INITIALIZE:
                funcName = "initialize";
                break;

            case missID::SET_POS_LOCAL:
                funcName = "setPosLocal";
                break;

            case missID::WAIT_KEY_PRESS:
                funcName = "waitKeyPress";
                break;

            case missID::SAVE_IMG:
                funcName = "saveImg";
                break;

            case missID::WAIT_TIME:
                funcName = "waitTime";
                break;

            case missID::WAIT_GPS:
                funcName = "waitGPS";
                break;

            case missID::WAIT_FOREVER:
                funcName = "waitForever";
                break;

            case missID::GP_TAKEOFF:
                funcName = "GPTakeoff";
                break;
            
            case missID::TAKEOFF_CUR:
                funcName = "takeoffCur";
                break;

            case missID::GP_LAND:
                funcName = "GPLand";
                break;

            case missID::LAND_CUR:
                funcName = "landCur";
                break;

            case missID::GEOTAG:
                funcName = "geotag";
                break;

            case missID::WAIT_WP:
                funcName = "waitWP";
                break;

            case missID::WAIT_CAM_TRIG:
                funcName = "waitCamTrig";
                break;

            case missID::WAIT_SAVE_IMG:
                funcName = "waitSaveImg";
                break;

            default:
                ROS_ERROR("Unkown mission function ID");
                return false;
                break;
        }

        ROS_INFO_STREAM("Invoking " << funcName);

        // Fill header data
        fillHeader(params.header, "mission_control");

        // Call service
        robot_msgs::RunMission runMissSrv;
        runMissSrv.request.func_id = missID;
        runMissSrv.request.params = params;
        if (missClient_.call(runMissSrv))
        {
            ROS_INFO_STREAM(funcName << " invoke successful");
        }
        else
        {
            ROS_ERROR_STREAM(funcName << " invoke failed");
        }

        // Clear mission params
        clearParams(params);

        return runMissSrv.response.success;
    }
}