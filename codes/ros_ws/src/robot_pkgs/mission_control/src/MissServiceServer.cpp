/**
 * This file is for mission functions server
 * Implemented using service architecture
 */

#include <mission_control/MissServiceServer.hpp>

namespace miss_service_server
{
    MissServiceServer::MissServiceServer() :
        missSrv_(nh_.advertiseService("mission_server", &MissServiceServer::serviceCB, this)) // advertise service server
    {
        ROS_INFO("Mission server initializing");

        // --- SET LOG VERBOSITY LEVEL ---

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

        // --- GET RELEVANT NAMESPACES ---

        // Get image processing server namespace
        if (nh_.getParam("/img_proc_ns", imgProcNs_) && imgProcNs_ != "")
        {
            imgProcNs_ = "/" + imgProcNs_;
            ROS_DEBUG_STREAM("Image processing namespace: " << imgProcNs_);
        }
        else
        {
            ROS_WARN("img_proc_ns param not found. Setting image processing namespace to \"\"");

            imgProcNs_ = "";
        }

        // Get MAVROS node namespace
        if (nh_.getParam("/mavros_ns", mavrosNs_) && mavrosNs_ != "")
        {
            mavrosNs_ = "/" + mavrosNs_;
            ROS_DEBUG_STREAM("MAVROS namespace: " << mavrosNs_);
        }
        else
        {
            ROS_WARN("mavros_ns param not found. Setting MAVROS namespace to \"\"");

            mavrosNs_ = "";
        }

        // Get GPIO node namespace
        if (nh_.getParam("/gpio_ns", GPIONs_) && GPIONs_ != "")
        {
            GPIONs_ = "/" + GPIONs_;
            ROS_DEBUG_STREAM("GPIO namespace: " << GPIONs_);
        }
        else
        {
            ROS_WARN("gpio_ns param not found, setting GPIO namespace to \"\"");

            GPIONs_ = "";
        }

        ROS_INFO_STREAM("Mission server initialized at: " << ros::this_node::getName());
    }

    // ----- PRIVATE METHODS -----

    // Function to process request and response message
    // Will be invoked for each service call
    bool MissServiceServer::serviceCB(robot_msgs::RunMission::Request& req, 
                                      robot_msgs::RunMission::Response& res)
    {
        // Info on which mission function to be run
        u_int8_t funcID {req.func_id};

        // Run mission
        uint8_t missRetVal;
        switch (funcID)
        {
            case Missions::INITIALIZE:
                ROS_INFO("Running: initialize");
                missRetVal = initialize(req.params);
                break;

            case Missions::ARM_THROTTLE:
                ROS_INFO("Running: armThrottle");
                missRetVal = armThrottle(req.params);
                break;

            case Missions::DISARM_THROTTLE:
                ROS_INFO("Running: disarmThrottle");
                missRetVal = disarmThrottle(req.params);
                break;
            
            case Missions::SET_POS_LOCAL:
                ROS_INFO("Running: setPosLocal");
                missRetVal = setPosLocal(req.params);
                break;

            case Missions::CHANGE_MODE:
                ROS_INFO("Running: changeMode");
                missRetVal = changeMode(req.params);
                break;

            case Missions::WAIT_KEY_PRESS:
                ROS_INFO("Running: waitKeyPress");
                missRetVal = waitKeyPress(req.params);
                break;

            case Missions::SAVE_IMG:
                ROS_INFO("Running: saveImg");
                missRetVal = saveImg(req.params);
                break;

            case Missions::WAIT_TIME:
                ROS_INFO("Running: waitTime");
                missRetVal = waitTime(req.params);
                break;

            case Missions::WAIT_GPS:
                ROS_INFO("Running: waitGPS");
                missRetVal = waitGPS(req.params);
                break;

            case Missions::WAIT_FOREVER:
                ROS_INFO("Running: waitForever");
                missRetVal = waitForever(req.params);
                break;

            case Missions::GP_TAKEOFF:
                ROS_INFO("Running: GPTakeoff");
                missRetVal = GPTakeoff(req.params);
                break;

            case Missions::TAKEOFF_CUR:
                ROS_INFO("Running: takeoffCur");
                missRetVal = takeoffCur(req.params);
                break;

            case Missions::LAND_CUR:
                ROS_INFO("Running: landCur");
                missRetVal = landCur(req.params);
                break;

            case Missions::GEOTAG:
                ROS_INFO("Running: geotag");
                missRetVal = geotag(req.params);
                break;

            case Missions::WAIT_WP:
                ROS_INFO("Running: waitWP");
                missRetVal = waitWP(req.params);
                break;

            case Missions::WAIT_CAM_TRIG:
                ROS_INFO("Running: waitCamTrig");
                missRetVal = waitCamTrig(req.params);
                break;

            case Missions::WAIT_SAVE_IMG:
                ROS_INFO("Running: waitSaveImg");
                missRetVal = waitSaveImg(req.params);
                break;

            default:
                ROS_ERROR("Unkown mission function ID");
                return false;
                break;
        }

        // Check return value and process accordingly
        if (missRetVal == Errors::NO_ERROR)
        {
            ROS_INFO("No error on mission function invoke");
            
            res.success = true;
            res.report = "No error";
            
            return true;
        }
        else // mission failed, parse according to return value
        {
            ROS_ERROR("Mission function invoke failed");

            std::string errorCause;
            switch (missRetVal)
            {
                case Errors::INVALID_ARGUMENT:
                    errorCause = "Invalid argument";
                    break;

                case Errors::SERVICE_NOT_AVAILABLE:
                    errorCause = "Service not available";
                    break;

                case Errors::SERVICE_CALL_FAILED:
                    errorCause = "Service call failed";
                    break;

                case Errors::FUNCTION_INVOKE_FAILED:
                    errorCause = "Function invoke failed";
                    break;

                case Errors::OTHERS:
                    errorCause = "Others";
                    break;

                case Errors::WAIT_TIMEOUT:
                    errorCause = "Waiting timeout";
                    break;

                default:
                    errorCause = "Error code unknown";
                    break;
            }

            ROS_ERROR_STREAM("Error cause: " << errorCause);

            res.success = false;
            res.report = "Error cause: " + errorCause;

            return false;
        }
    }

    // ----- MISSION FUNCTIONS -----

    /**
     * Function to initialize UAV
     * 
     * Params: -
     * 
     * NOTES : - Unfinished
     */
    uint8_t MissServiceServer::initialize(const Params& params)
    {
        ROS_INFO("--- initialize ---");

        ROS_INFO("Initializing robot");

        // --- PUBLISH SETPOINT ---

        ROS_DEBUG("Publishing setpoints");

        // For storing setpoint data
        Params setPosLocalParams;
        setPosLocalParams.floats.push_back(0);
        setPosLocalParams.floats.push_back(0);
        setPosLocalParams.floats.push_back(0);

        // Send a few setpoints so that code can be started
        for (u_int8_t count{100}; ros::ok() && count > 0; --count)
        {
            setPosLocal(setPosLocalParams);

            // ArduPilot refreshes command from companion computer every 500 ms
            // Hence, the refresh rate here must be set at at least 2 Hz
            ros::Rate(10).sleep();
        }

        ROS_DEBUG("Setpoints published");

        // --- SET HOME LOCATION ---

        ROS_DEBUG("Setting HOME location");

        // Read and store GPS data
        boost::shared_ptr<const sensor_msgs::NavSatFix> GPSDataPtr {ros::topic::waitForMessage<sensor_msgs::NavSatFix>(mavrosNs_ + "/mavros/global_position/global")};
        sensor_msgs::NavSatFix GPSData {*GPSDataPtr};

        const float lat{GPSData.latitude};
        const float lon{GPSData.longitude};
        const float alt{GPSData.altitude};

        // Set HOME service client
        ros::ServiceClient homeClient {nh_.serviceClient<mavros_msgs::CommandHome>(mavrosNs_ + "/mavros/cmd/set_home")};

        // Service data
        mavros_msgs::CommandHome homeSrv;
        homeSrv.request.current_gps     = true;
        homeSrv.request.latitude        = lat;
        homeSrv.request.longitude       = lon;
        homeSrv.request.altitude        = alt;

        // Wait until set HOME service exists
        while (!homeClient.exists())
        {
            ;
        }

        // Call set HOME service until successful
        while (!homeClient.call(homeSrv) || !homeSrv.response.success)
        {
            ;
        }

        ROS_DEBUG("HOME position set");
        
        // --- SET GPS ORIGIN ---

        ROS_DEBUG("Setting GPS origin");
        
        // Publisher for GPS origin
        ros::Publisher GPSOriginPub{ nh_.advertise<geographic_msgs::GeoPointStamped>(mavrosNs_ + "/mavros/global_position/set_gp_origin", 1) };
        geographic_msgs::GeoPointStamped GPSOrigin;

        // Message data for GPS origin
        GPSOrigin.header.stamp		    = ros::Time::now();
		GPSOrigin.header.frame_id		= "gps_origin";
		GPSOrigin.position.latitude 	= lat;
		GPSOrigin.position.longitude 	= lon;
		GPSOrigin.position.altitude 	= alt;

        // Wait until GPS origin subscriber is present
        while ( GPSOriginPub.getNumSubscribers() <= 0 )
		{
			;
		}

        // Publish GPS origin message
        GPSOriginPub.publish(GPSOrigin);

        ROS_DEBUG("GPS origin set");

        return Errors::NO_ERROR;
    }

    /**
     * Function to arm robot throttle
     * 
     * Params: -
     */
    uint8_t MissServiceServer::armThrottle(const Params& params)
    {
        ROS_INFO("--- armThrottle ---");

        ROS_INFO("Arming robot throttle");

        // Call service
        ros::ServiceClient armClient {nh_.serviceClient<mavros_msgs::CommandBool>(mavrosNs_ + "/mavros/cmd/arming")};
        if (armClient.exists()) // service exists
        {
            mavros_msgs::CommandBool armSrv;
            armSrv.request.value = true;

            if (armClient.call(armSrv) && armSrv.response.success) // service call successful
            {
                ROS_INFO("Arm service call succesful");
                if (armSrv.response.result) // arming successful
                {
                    ROS_INFO("Arming successful");
            
                    return Errors::NO_ERROR;
                }
                
                // If it reached this point, it means arming failed
                ROS_ERROR("Arming failed");

                return Errors::OTHERS;
            }
            else // service call failed
            {
                ROS_ERROR("Arm service call failed");
            
                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Arm service doesn't exist");
            
            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to disarm robot throttle
     * 
     * Params: -
     */
    uint8_t MissServiceServer::disarmThrottle(const Params& params)
    {
        ROS_INFO("--- disarmThrottle ---");

        ROS_INFO("Disarming robot throttle");

        // Call service
        ros::ServiceClient disarmClient {nh_.serviceClient<mavros_msgs::CommandBool>(mavrosNs_ + "/mavros/cmd/arming")};
        if (disarmClient.exists()) // service exists
        {
            mavros_msgs::CommandBool disarmSrv;
            disarmSrv.request.value = false;

            if (disarmClient.call(disarmSrv) && disarmSrv.response.success) // service call successful
            {
                ROS_INFO("Disarm service call succesful");
                if (disarmSrv.response.result) // disarming successful
                {
                    ROS_INFO("Disarming successful");

                    return Errors::NO_ERROR;
                }
            
                // If it reached this point, it means disarming failed
                ROS_ERROR("Disrming failed");

                return Errors::OTHERS;
            }
            else // service call failed
            {
                ROS_ERROR("Disarm service call failed");
            
                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Disarm service doesn't exist");
            
            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to move robot with setpoint_position/local
     * 
     * Params: - floats[0] = point.x
     *         - floats[1] = point.y
     *         - floats[2] = point.z
     *         - floats[3] = quaternion.x (optional)
     *         - floats[4] = quaternion.y (optional)
     *         - floats[5] = quaternion.z (optional)
     *         - floats[6] = quaternion.w (optional)
     * 
     * Notes:
     * - Quaternion messages will only be sent if all quaternion element is complete
     * - See also: http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
     */
    uint8_t MissServiceServer::setPosLocal(const Params& params)
    {
        ROS_INFO("--- setPosLocal ---");

        // Check arguments
        if (params.floats.size() < 3)
        {
            ROS_ERROR("Number of floats arguments passed to setPosLocal too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store params data
        const double pointX{static_cast<double>(params.floats[0])};
        const double pointY{static_cast<double>(params.floats[1])};
        const double pointZ{static_cast<double>(params.floats[2])};
        double quatX;
        double quatY;
        double quatZ;
        double quatW;

        // For publishing setpoint message
        ros::Publisher pub {nh_.advertise<geometry_msgs::PoseStamped>(mavrosNs_ + "/mavros/setpoint_position/local", 1)};

        // For storing setpoint data
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pointX;
        pose.pose.position.y = pointY;
        pose.pose.position.z = pointZ;

        ROS_INFO("Will move to (%.2f, %.2f, %.2f)", 
                 pointX, pointY, pointZ);

        // If all quaternion elements are complete, send the message
        if (params.floats.size() >= 7)
        {
            // Store quaternion data
            quatX = static_cast<double>(params.floats[3]);
            quatY = static_cast<double>(params.floats[4]);
            quatZ = static_cast<double>(params.floats[5]);
            quatW = static_cast<double>(params.floats[6]);

            // Message data
            pose.pose.orientation.x = quatX;
            pose.pose.orientation.y = quatY;
            pose.pose.orientation.z = quatZ;
            pose.pose.orientation.w = quatW;

            ROS_INFO("Robot orientation will be (%.2f, %.2f, %.2f, %.2f)",
                     quatX, quatY, quatZ, quatW);
        }

        // Publish message when there are subscribers
        while (pub.getNumSubscribers() < 1)
        {
            ;
        }
        pub.publish(pose);

        ROS_INFO("%s/mavros/setpoint_position/local messsage has been published", mavrosNs_.c_str());

        return Errors::NO_ERROR;
    }

    /**
     * Function to change flight mode
     * 
     * Params: - strings[0] = target flight mode
     * 
     * Notes:
     * - See also: http://wiki.ros.org/mavros/CustomModes
     */
    uint8_t MissServiceServer::changeMode(const Params& params)
    {
        ROS_INFO("--- changeMode ---");

        // Check argument size
        if (params.strings.size() < 1)
        {
            ROS_ERROR("Number of strings arguments passed to changeMode too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store param data
        const std::string mode(params.strings[0]);

        ROS_INFO_STREAM("Changing mode to " << mode);

        // Call service
        ros::ServiceClient setModeClient {nh_.serviceClient<mavros_msgs::SetMode>(mavrosNs_ + "/mavros/set_mode")};
        if (setModeClient.exists()) // service exists
        {
            mavros_msgs::SetMode setModeSrv;
            setModeSrv.request.base_mode = 0;
            setModeSrv.request.custom_mode = params.strings[0];

            if (setModeClient.call(setModeSrv) && setModeSrv.response.mode_sent) // service call successful
            {
                ROS_INFO("Mode change service call succesful");
                if (setModeSrv.response.mode_sent)
                {
                    ROS_INFO("Mode change successful");

                    return Errors::NO_ERROR;
                }
            
                // If it reached this point, it means mode are not recognized or not sent
                ROS_ERROR("Mode change failed");

                return Errors::OTHERS;
            }
            else // service call failed
            {
                ROS_ERROR("Mode change service call failed");
            
                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Mode change service doesn't exist");
            
            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to wait until a certain key on keyboard has been pressed
     * 
     * Params: - uints[0] = key press code to wait for
     *         - floats[0] = timeout duration in seconds, <= 0 means infinite (optional)
     *           default: infinite
     * 
     * Notes:
     * - See also: key code value at Key.message in keyboard package (https://github.com/lrse/ros-keyboard)
     * - Somehow topic value can't be extracted, although has been subscribed to that topic
     */
    uint8_t MissServiceServer::waitKeyPress(const Params& params)
    {
        ROS_INFO("--- waitKeyPress ---");

        // Check and store arguments
        if (params.uints.size() < 1)
        {
            ROS_ERROR("Number of uints arguments passed to waitKeyPress too little");

            return Errors::INVALID_ARGUMENT;
        }
        const uint16_t desiredKey{params.uints[0]};

        ros::Duration timeoutDur{ros::DURATION_MAX};
        if (params.floats.size() >= 1)
        {
            timeoutDur = ros::Duration(params.floats[0]);
        }

        ROS_INFO_STREAM("Waiting for " << desiredKey << " to be pressed");

        // Get keyboard node namespace
        std::string keyboardNs;
        if (nh_.getParam("/keyboard_ns", keyboardNs))
        {
            ROS_DEBUG_STREAM("Keyboard node namespace: " << keyboardNs);
        }
        else
        {
            ROS_WARN("Keyboard node namespace not found, setting to default");
            ROS_WARN("Keyboard node namespace is set to \"\"");
            
            keyboardNs = "";
        }

        // Store keyboard topic
        const std::string keyDownTopic("/" + keyboardNs + "/keyboard/keydown");

        // Wait until desired key is pressed or timeout reached
        ros::Time startTime {ros::Time::now()};
        for (bool correctKey{false}, timeoutReached{false}; !correctKey && !timeoutReached; )
        {
            // For reading pressed key
            boost::shared_ptr<const keyboard::Key> keyPtr;
            keyboard::Key keyPress;

            // Checks for pressed key
			while ( keyPtr == NULL && !correctKey && !timeoutReached )
			{
				keyPtr = ros::topic::waitForMessage<keyboard::Key>(keyDownTopic);

                keyPress = *keyPtr;

                if (keyPress.code == desiredKey)
                {
                    ROS_INFO("Desired key pressed");

                    correctKey = true;
                }
                else if (ros::Time::now() - startTime >= timeoutDur)
                {
                    ROS_INFO("Timeout reached");

                    timeoutReached = true;
                }
                else
                {
                    ROS_DEBUG("%i pressed", keyPress.code);
                }
			}

            ros::Rate(10).sleep();
        }

        return Errors::NO_ERROR;
    }

    /**
     * Function to save image from video stream
     * 
     * Params: - strings[0] = save location, relative to $HOME
     *         - strings[1] = image name
     */
    uint8_t MissServiceServer::saveImg(const Params& params)
    {
        ROS_INFO("--- saveImg ---");

        // Check arguments
        if (params.strings.size() < 2)
        {
            ROS_ERROR("Number of strings arguments passed into saveImg too little");
        }

        // Image processing service client
        ros::ServiceClient imgProcClient {nh_.serviceClient<robot_msgs::ProcessImage>(imgProcNs_ + "/img_proc_server")};

        // Check server existence
        if (imgProcClient.exists()) // service exists
        {
            // Get value of $HOME
            std::string HOME;
            const char* envVar {std::getenv("HOME")};
            if (envVar == nullptr)
            {
                HOME = "";
            }
            else
            {
                HOME = envVar;
            }

            // Store image save location and file name
            const std::string saveLoc(HOME + "/" + params.strings[0] + "/");
            const std::string fileName(params.strings[1]);

            ROS_DEBUG_STREAM("HOME is set to " << HOME);

            ROS_INFO_STREAM("Will save latest image to " << saveLoc + fileName);

            // Store process image service request data
            robot_msgs::ProcessImage procImgSrv;
            procImgSrv.request.func_id = robot_msgs::ImgProcMap::SAVE_IMG;
            procImgSrv.request.params.strings.push_back(saveLoc);
            procImgSrv.request.params.strings.push_back(fileName);

            // Call service
            if (imgProcClient.call(procImgSrv) && procImgSrv.response.success) // service call successful
            {
                // Camera feedback logging

                // GPIO service client
                ros::ServiceClient GPIOClient {nh_.serviceClient<robot_msgs::GPIO>(GPIONs_ + "gpio_server")};

                // Check server existence
                if (GPIOClient.exists()) // service exists
                {
                    // Store GPIO service request data
                    robot_msgs::GPIO GPIOSrv;
                    GPIOSrv.request.func_id = robot_msgs::GPIOMap::CAM_FEEDBACK;

                    // Call service
                    if (GPIOClient.call(GPIOSrv) && GPIOSrv.response.success) // service call successful
                    {
                        ROS_INFO("saveImg successful");

                        return Errors::NO_ERROR;
                    }
                    else // service call failed
                    {
                        ROS_ERROR("saveImg failed");

                        return Errors::SERVICE_CALL_FAILED;
                    }
                }
                else // server doesn't exist
                {
                    ROS_ERROR("GPIO service doesn't exist");

                    return Errors::SERVICE_NOT_AVAILABLE;
                }

                ROS_INFO("saveImg successful");

                return Errors::NO_ERROR;
            }
            else // service call failed
            {
                ROS_ERROR("saveImg failed");

                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Image processing service doesn't exist");

            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to wait until a set duration
     * 
     * Params: - floats[0] = seconds
     *         - strings[0] = mode to change into after waiting (optional)
     *           default: GUIDED
     */
    uint8_t MissServiceServer::waitTime(const Params& params)
    {
        ROS_INFO("--- waitTime ---");

        // Check and store arguments
        if (params.floats.size() < 1)
        {
            ROS_ERROR("Number of floats arguments passed into waitTime too little");
        }
        float secs{params.floats[0]};

        std::string desiredMode("GUIDED");
        if (params.strings.size() >= 1)
        {
            desiredMode = params.strings[0];
        }

        ROS_INFO_STREAM("Will wait for " << 
                        secs << " seconds");

        // Change into BRAKE mode, and then wait
        Params chgModeParams;
        chgModeParams.strings.push_back("BRAKE");
        changeMode(chgModeParams);

        // Wait
        ros::Duration(static_cast<double>(secs)).sleep();

        // Change to desired mode
        chgModeParams.strings.clear();
        chgModeParams.strings.push_back(desiredMode);
        changeMode(chgModeParams);

        ROS_INFO("Wait finished");

        return Errors::NO_ERROR;
    }

    /**
     * Function to wait until robot reached certain GPS coordinate
     * 
     * Params: - floats[0] = latitude
     *         - floats[1] = longitude
     *         - floats[2] = altitude
     *         - uints[0] = tolerance, larger values means more tolerant (optional)
     *                      default: 100
     * 
     * Notes:
     * - Beware of floating points operation
     */
    uint8_t MissServiceServer::waitGPS(const Params& params)
    {
        ROS_INFO("--- waitGPS ---");

        // Check arguments
        if (params.floats.size() < 3)
        {
            ROS_ERROR("Number of floats arguments passed to waitGPS too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store data from params
        const double lat {static_cast<double>(params.floats[0])};
        const double lon {static_cast<double>(params.floats[1])};
        const double alt {static_cast<double>(params.floats[2])};
        uint32_t tol;

        ROS_INFO("Wait until robot reach (%.2f, %.2f, %.2f)", 
                 lat, lon, alt);

        if (params.uints.size() >= 1)
        {
            tol = params.uints[0];
        }
        else
        {
            tol = 100;
        }

        // For storing GPS message data
        sensor_msgs::NavSatFix GPSData;

        // Callback function for getting GPS data
        boost::function<void(const sensor_msgs::NavSatFix&)> GPCB
        {
            [&GPSData](const sensor_msgs::NavSatFix& msg)
            {
                GPSData = msg;
            }
        };

        // Subscribe to GPS data topic
        ros::Subscriber GPSDataSub {nh_.subscribe<sensor_msgs::NavSatFix>(mavrosNs_ + "/mavros/global_position/global",
                                                                          1,
                                                                          GPCB)};

        // Wait until robot reach desired coordinates
        bool reached {false};
        while (!reached)
        {
            // Will only break out of loop if coordinate has been reached
            if (abs(GPSData.latitude - lat)*(1/tol) <= 1  &&
                abs(GPSData.longitude - lon)*(1/tol) <= 1 &&
                abs(GPSData.altitude - alt)*(1/tol) <= 1  )
            {
                reached = true;
            }

            ros::Rate(10).sleep();
        }

        ROS_INFO("(%4.2f, %4.2f, %4.2f) reached",
                 params.floats[0], 
                 params.floats[1], 
                 params.floats[2]);

        return Errors::NO_ERROR;
    }

    /**
     * Function to wait forever
     * 
     * Params: -
     * 
     * Notes:
     * - This function will really make the robot do nothing, until this node is shut down
     */
    uint8_t MissServiceServer::waitForever(const Params& params)
    {
        ROS_INFO("--- waitForever ---");

        ROS_INFO("Robot will now wait until node is shut down");

        // Change into BRAKE mode, and then wait
        Params chgModeParams;
        chgModeParams.strings.push_back("BRAKE");
        changeMode(chgModeParams);

        while (true)
        {
            ;
        }

        return Errors::NO_ERROR; // kinda useless, but yeap
    }

    /**
     * Function to initiate takeoff with global coordinates
     * 
     * Params: - floats[0] = latitutde
     *         - floats[1] = longitude
     *         - floats[2] = altitude
     *         - floats[3] = minimal pitch (optional)
     *         - floats[4] = yaw (optional)
     */
    uint8_t MissServiceServer::GPTakeoff(const Params& params)
    {
        ROS_INFO("--- GPTakeoff ---");

        ROS_INFO("Initiating GPTakeoff");

        if (params.floats.size() < 3)
        {
            ROS_ERROR("Number of floats arguments passed into GPTakeoff too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store params data
        const float lat{params.floats[0]};
        const float lon{params.floats[1]};
        const float alt{params.floats[2]};
        float minPitch{0};
        float yaw{0};

        if (params.floats.size() >= 5)
        {
            minPitch = params.floats[3];
            yaw = params.floats[4];
        }
        else if (params.floats.size() >= 4)
        {
            minPitch = params.floats[3];
        }

        // Call service
        ros::ServiceClient TOClient {nh_.serviceClient<mavros_msgs::CommandTOL>(mavrosNs_ + "/mavros/cmd/takeoff")};
        if (TOClient.exists()) // service exists
        {
            mavros_msgs::CommandTOL TOSrv;
            TOSrv.request.altitude  = alt;
            TOSrv.request.latitude  = lat;
            TOSrv.request.longitude = lon;
            TOSrv.request.min_pitch = minPitch;
            TOSrv.request.yaw       = yaw;

            if (TOClient.call(TOSrv) && TOSrv.response.success) // service call successful
            {
                ROS_INFO("Takeoff service call succesful");
            
                return Errors::NO_ERROR;
            }
            else // service call failed
            {
                ROS_ERROR("Takeoff service call failed");
            
                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Takeoff service doesn't exist");
            
            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to takeoff in indoor environment or with current latitude and longitude
     * 
     * Params: - floats[0] = altitude, in meters
     */
    uint8_t MissServiceServer::takeoffCur(const Params& params)
    {
        ROS_INFO("--- takeoffCur ---");

        if (params.floats.size() < 1)
        {
            ROS_ERROR("Number of floats arguments passed into takeoffCur too little");

            return Errors::INVALID_ARGUMENT;
        }

        const float alt{params.floats[0]};

        ROS_INFO("Will try to takeoff to %.2f meter", alt);

        // Publish setpoint local topic
        Params setPosLocParams;
        setPosLocParams.floats.push_back(0);
        setPosLocParams.floats.push_back(0);
        setPosLocParams.floats.push_back(alt);
        return setPosLocal(setPosLocParams);
    }

    /**
     * Function to land with desired latitude and longitude
     * 
     * Params: - floats[0] = latitude
     *         - floats[1] = longitude
     *         - floats[2] = yaw (optional)
     */
    uint8_t MissServiceServer::GPLand(const Params& params)
    {
        ROS_INFO("--- GPLand ---");

        if (params.floats.size() < 2)
        {
            ROS_ERROR("Number of floats arguments passed into GPLand too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store params data
        const float lat{params.floats[0]};
        const float lon{params.floats[1]};
        const float alt{0};
        float yaw{0};

        ROS_INFO("Will try to land on (%.2f, %.2f, %.2f)", 
                 lat, lon, alt);

        if (params.floats.size() >= 3)
        {
            yaw = params.floats[2];
        }

        // Call service
        ros::ServiceClient landClient {nh_.serviceClient<mavros_msgs::CommandTOL>(mavrosNs_ + "/mavros/cmd/land")};
        if (landClient.exists()) // service exists
        {
            mavros_msgs::CommandTOL landSrv;
            landSrv.request.altitude  = alt;
            landSrv.request.latitude  = lat;
            landSrv.request.longitude = lon;
            landSrv.request.yaw       = yaw;

            if (landClient.call(landSrv) && landSrv.response.success) // service call successful
            {
                ROS_INFO("Land service call succesful");
            
                return Errors::NO_ERROR;
            }
            else // service call failed
            {
                ROS_ERROR("Land service call failed");
            
                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Land service doesn't exist");
            
            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to land in indoor environment or current latitude and longitude
     * 
     * Params: -
     */
    uint8_t MissServiceServer::landCur(const Params& params)
    {
        ROS_INFO("--- landCur ---");

        // Switch to land mode
        Params landParams;
        landParams.strings.push_back("LAND");
        return changeMode(landParams);
    }

    /**
     * Function to geotag image
     * 
     * Params: - strings[0] = save location, relative to $HOME
     *         - strings[1] = image file name, with file extension
     *         - strings[2] = CSV file name, with file extension
     */
    uint8_t MissServiceServer::geotag(const Params& params)
    {
        ROS_INFO("--- geotag ---");

        // Check params
        if (params.strings.size() < 3)
        {
            ROS_ERROR("Number of strings param passed into geotag too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Image processing service client
        ros::ServiceClient geoPosClient {nh_.serviceClient<robot_msgs::GeoProcessing>(imgProcNs_ + "/geo_pos_server")};

        // Check server existence
        if (geoPosClient.exists()) // service exists
        {
            // Get value of $HOME
            std::string HOME;
            const char* envVar {std::getenv("HOME")};
            if (envVar == nullptr)
            {
                HOME = "";
            }
            else
            {
                HOME = envVar;
            }

            // Store image save location and file name
            const std::string folderLoc(HOME + "/" + params.strings[0] + "/");
            const std::string imgName(params.strings[1]);
            const std::string CSVName(params.strings[2]);

            ROS_DEBUG_STREAM("HOME is set to " << HOME);

            ROS_INFO_STREAM("Will geotag image in " << folderLoc + imgName);

            // Store process image service request data
            robot_msgs::GeoProcessing geoPosSrv;
            geoPosSrv.request.func_id = robot_msgs::GeoProcMap::GEOTAG;
            geoPosSrv.request.params.strings.push_back(folderLoc);
            geoPosSrv.request.params.strings.push_back(imgName);
            geoPosSrv.request.params.strings.push_back(CSVName);

            // Call service
            if (geoPosClient.call(geoPosSrv) && geoPosSrv.response.success) // service call successful
            {
                ROS_INFO("geotag successful");

                return Errors::NO_ERROR;
            }
            else // service call failed
            {
                ROS_ERROR("geotag failed");

                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // service doesn't exist
        {
            ROS_ERROR("Image processing service doesn't exist");

            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to make robot wait until certain waypoint is reached
     * Will change mode into AUTO
     * 
     * Params: - uints[0] = mission sequence number
     */
    uint8_t MissServiceServer::waitWP(const Params& params)
    {
        ROS_INFO("--- waitWP ---");

        // Check params
        if (params.uints.size() < 1)
        {
            ROS_ERROR("Number of strings param passed into waitWP too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store param data
        const uint16_t seqNo{static_cast<uint16_t>(params.uints[1])};

        ROS_INFO("Will wait until waypoint with sequence number %u is reached", seqNo);

        // For storing waypoint list
        mavros_msgs::WaypointList WPList;

        // Callback function to get waypoint list
        boost::function<void(const mavros_msgs::WaypointList&)> WPListCB
        {
            [&WPList](const mavros_msgs::WaypointList& msg)
            {
                WPList.current_seq = msg.current_seq;
            }
        };

        // Change mode into AUTO, and then wait until desired waypoint is reached
        Params modeParams;
        modeParams.strings.push_back("AUTO");
        changeMode(modeParams);

        // Wait until desired waypoint is reached
        ROS_INFO("Waiting...");
        while (WPList.current_seq < seqNo)
        {
            ros::Rate(10).sleep();
        }
        ROS_INFO("Waypoint reached");

        return Errors::NO_ERROR;
    }

    /**
     * Function to wait until camera trigger command has been received by SBC
     * 
     * Params: - floats[0] = timeout duration in seconds, <= 0 means infinite (optional)
     *           default: infinite
     */
    uint8_t MissServiceServer::waitCamTrig(const Params& params)
    {
        ROS_INFO("--- waitCamTrig ---");

        // Store param data
        float timeoutDur{0};
        if (params.floats.size() >= 1)
        {
            timeoutDur = params.floats[0];
        }

        // GPIO service client
        ros::ServiceClient GPIOClient {nh_.serviceClient<robot_msgs::GPIO>(GPIONs_ + "/gpio_server")};

        ROS_DEBUG_STREAM("GPIO server address: " << GPIONs_ + "/gpio_server");
        
        // Check server existence
        if (GPIOClient.exists()) // service exists
        {
            // Store GPIO service request data
            robot_msgs::GPIO GPIOSrv;
            GPIOSrv.request.func_id = robot_msgs::GPIOMap::WAIT_CAM_TRIG;
            GPIOSrv.request.params.floats.push_back(timeoutDur);

            // Call service
            if (GPIOClient.call(GPIOSrv) && GPIOSrv.response.success) // service call successful
            {
                ROS_INFO("waitCamTrig successful");

                return Errors::NO_ERROR;
            }
            else // service call failed
            {
                ROS_ERROR("waitCamTrig failed");

                return Errors::SERVICE_CALL_FAILED;
            }
        }
        else // server doesn't exist
        {
            ROS_ERROR("GPIO service doesn't exist");

            return Errors::SERVICE_NOT_AVAILABLE;
        }
    }

    /**
     * Function to wait until camera trigger command has been received by SBC
     * And then save image from video stream
     * 
     * Params: - floats[0] = timeout duration in seconds, <= 0 means infinite (optional)
     *           default: infinite
     *         - strings[0] = save location, relative to $HOME
     *         - strings[1] = image name
     */
    uint8_t MissServiceServer::waitSaveImg(const Params& params)
    {
        ROS_INFO("--- waitSaveimg ---");

        // Wait for trigger command
        uint8_t retVal{waitCamTrig(params)};
        if (retVal == Errors::NO_ERROR)
        {
            // If trigger command is received, save image
            retVal = saveImg(params);
            if (retVal == Errors::NO_ERROR)
            {
                ROS_INFO("waitSaveImg successful");

                return retVal;
            }
            else // saveImg failed
            {
                ROS_ERROR("waitSaveImg failed");

                return retVal;
            }
        }
        else // no trigger command received
        {
            return retVal;
        }
    }

    // ----- PUBLIC METHODS -----

}