/**
 * Class for GPIO related stuffs
 */

#include <gpio/GPIO.hpp>

namespace gpio
{
    GPIO::GPIO() :
        GPIOSrv_(nh_.advertiseService("gpio_server", &GPIO::serviceCB, this)) // advertise service server
    {
        ROS_INFO("GPIO server initializing");
        
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

        // Get GPIO server namespace
        std::string GPIONs;
        if (nh_.getParam("/gpio_ns", GPIONs) && GPIONs != "")
        {
            GPIONs = "/" + GPIONs;
            ROS_DEBUG_STREAM("GPIO namespace: " << GPIONs);
        }
        else
        {
            ROS_WARN("gpio_ns param not found, setting GPIO namespace to \"\"");

            GPIONs = "";
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

        // --- GET GPIO CONFIG PARAMS ---

        // Camera trigger pin
        if (!nh_.getParam(GPIONs + "/cam_trig_pin", camTrigPin_))
        {
            ROS_WARN_STREAM(GPIONs << "/cam_trig_pin not found. Setting camera trigger pin to 4");

            camTrigPin_ = 4;
        }

        // Camera feedback pin
        if (!nh_.getParam(GPIONs + "/cam_feed_pin", camFeedPin_))
        {
            ROS_WARN_STREAM(GPIONs << "/cam_feed_pin not found. Setting camera feedback pin to 5");

            camFeedPin_ = 5;
        }

        // --- GET AND STORE CAM PARAMS FROM FCU ---

        // Subscribe to get param service
        ros::ServiceClient getFCUParam {nh_.serviceClient<mavros_msgs::ParamGet>(mavrosNs_ + "/mavros/param/get")};

        // Store service request data
        mavros_msgs::ParamGet paramGetSrv;

        // Get CAM_DURATION
        paramGetSrv.request.param_id = "CAM_DURATION";
        if (getFCUParam.call(paramGetSrv) && paramGetSrv.response.success)
        {
            CAM_DURATION_ = paramGetSrv.response.value.integer;
        }

        // Get CAM_FEEDBACK_POL
        paramGetSrv.request.param_id = "CAM_FEEDBACK_POL";
        if (getFCUParam.call(paramGetSrv) && paramGetSrv.response.success)
        {
            CAM_FEEDBACK_POL_ = paramGetSrv.response.value.integer;
        }

        // Get CAM_RELAY_ON
        paramGetSrv.request.param_id = "CAM_RELAY_ON";
        if (getFCUParam.call(paramGetSrv) && paramGetSrv.response.success)
        {
            CAM_RELAY_ON_ = paramGetSrv.response.value.integer;
        }

        // --- WIRINGPI SETUP ---

        // Using WiringPi pin numbering
        wiringPiSetup();

        /**
         * Configure GPIO pins
         * All digital input pins will be pulled to 3.3v
         */

        // Pin for getting trigger signal
        pinMode(camTrigPin_, INPUT);
        pullUpDnControl(camTrigPin_, PUD_UP);

        // Pin for camera feedback logging
        pinMode(camFeedPin_, OUTPUT);
        digitalWrite(camFeedPin_, !CAM_FEEDBACK_POL_);

        ROS_INFO_STREAM("GPIO server initialized at: " << ros::this_node::getName());
    }

    // ----- PRIVATE METHODS -----

    // ----- CALLBACKS -----

    bool GPIO::serviceCB(robot_msgs::GPIO::Request& req, 
                         robot_msgs::GPIO::Response& res)
    {
        // Info on which funciton to be run
        uint8_t funcID {req.func_id};

        // Run image processing function
        int funcRetVal;
        switch (funcID)
        {
            case Funcs::WAIT_CAM_TRIG:
                ROS_INFO("Running: camTriggered");
                funcRetVal = waitCamTrig(req.params);
                break;

            case Funcs::CAM_FEEDBACK:
                ROS_INFO("Running: camFeedback");
                funcRetVal = camFeedback(req.params);
                break;

            default:
                ROS_ERROR("Unknown ID function ID passed");
                return false;
                break;
        }

        // Check return value and process accordingly
        if (funcRetVal == Errors::NO_ERROR)
        {
            ROS_INFO("No error on mission function invoke");
            
            res.success = true;
            res.report = "No error";
            
            return true;
        }
        else
        {
            ROS_ERROR("GPIO function invoke failed");

            std::string errorCause;
            switch (funcRetVal)
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

    // ----- GPIO FUNCTIONS -----

    /**
     * Function to check if camera trigger command has been sent from FCU
     * 
     * Params: - floats[0] = timeout duration in seconds, <= 0 means infinite (optional)
     *           default: indefinitely
     * 
     * Notes: - Pin for receiving trigger command must be set into digital input
     *        - CAM_TRIGG_TYPE on ArduPilot must be set to 1 or Relay
     *        - RELAY_DEFAULT and CAM_RELAY_ON must be opposite values
     */
    uint8_t GPIO::waitCamTrig(const Params& params)
    {
        ROS_INFO("--- waitCamTrig ---");

        // Check and store param data
        uint32_t timeoutDur;
        if (params.floats.size() >= 1)
        {
            if (params.floats[0] <= 0)
            {
                timeoutDur = 4294967295; // largest value of uint32_t
            }
            else
            {
                timeoutDur = static_cast<uint32_t>(params.floats[0]*1000);
            }
        }
        else // wait indefinitely
        {
            timeoutDur = 4294967295; // largest value of uint32_t
        }

        ROS_DEBUG_STREAM("CAM_DURATION: " << CAM_DURATION_);
        ROS_DEBUG_STREAM("CAM_FEEDBACK_POL: " << CAM_FEEDBACK_POL_);
        ROS_DEBUG_STREAM("CAM_RELAY_ON: " << CAM_RELAY_ON_);

        // Check if camera shutter command is sent
        enum class Status
        {
            WAIT_FIRST_SIGNAL, // for when waiting for low signal from FCU
            CHECK_SIGNAL // for checking signal validity
        };
        Status status{Status::WAIT_FIRST_SIGNAL};
        bool signalSent{false};
        bool cmdSent{false};
        uint32_t sigSentTime;
        uint32_t startTime{millis()};
        uint32_t currTime{millis()};
        uint32_t deltaTime{0};
        switch (CAM_RELAY_ON_)
        {
            ROS_DEBUG("Testing value of CAM_RELAY_ON");
            case 0: // command sent as low
            {
                for ( ; 
                     (currTime - startTime <= timeoutDur) && !cmdSent;
                     (currTime = millis()), (deltaTime = currTime - startTime))
                {
                    switch (status)
                    {
                        case Status::WAIT_FIRST_SIGNAL:
                        {
                            if (digitalRead(camTrigPin_) == LOW)
                            {
                                status = Status::CHECK_SIGNAL;
                            }

                            break;
                        }

                        case Status::CHECK_SIGNAL:
                        {
                            if (digitalRead(camTrigPin_) == HIGH)
                            {
                                status = Status::CHECK_SIGNAL;
                                cmdSent = false;
                                signalSent = false;
                            }

                            // Store sent signal time data
                            if (!signalSent)
                            {
                                signalSent = true;
                                sigSentTime = millis();
                            }

                            // Check signal validity
                            if (signalSent && ((millis() - sigSentTime) >= (0.9*10*CAM_DURATION_)))
                            {
                                cmdSent = true;
                            }

                            break;
                        }
                    }

                    ros::Rate(100).sleep(); // update rate of relay pin is 50 Hz, must be faster than that
                }

                break;
            }

            case 1: // command sent as high
            {
                for ( ; 
                     (currTime - startTime <= timeoutDur) && !cmdSent;
                     (currTime = millis()), (deltaTime = currTime - startTime))
                {
                    switch (status)
                    {
                        case Status::WAIT_FIRST_SIGNAL:
                        {
                            if (digitalRead(camTrigPin_) == HIGH)
                            {
                                status = Status::CHECK_SIGNAL;
                            }

                            break;
                        }

                        case Status::CHECK_SIGNAL:
                        {
                            if (digitalRead(camTrigPin_) == LOW)
                            {
                                status = Status::CHECK_SIGNAL;
                                cmdSent = false;
                                signalSent = false;
                            }

                            // Store sent signal time data
                            if (!signalSent)
                            {
                                signalSent = true;
                                sigSentTime = millis();
                            }

                            // Check signal validity
                            if (signalSent && ((millis() - sigSentTime) >= (0.9*100*CAM_DURATION_)))
                            {
                                cmdSent = true;
                            }

                            break;
                        }
                    }

                    ros::Rate(100).sleep(); // update rate of relay pin is 50 Hz, must be faster than that
                }

                break;
            }

            if (cmdSent) // camera trigger command is sent
            {
                return Errors::NO_ERROR;
            }
            else // camera trigger command is not sent
            {
                return Errors::WAIT_TIMEOUT;
            }
        }
    }

    /**
     * Function to send camera trigger feedback signal to FCU
     * 
     * Params: -
     */
    uint8_t GPIO::camFeedback(const Params& params)
    {
        constexpr uint16_t duration{2};

        uint16_t startTime{millis()};
        digitalWrite(camFeedPin_, !CAM_FEEDBACK_POL_);
        while (millis() - startTime <= duration)
        {
            digitalWrite(camFeedPin_, CAM_FEEDBACK_POL_);
        }
        digitalWrite(camFeedPin_, !CAM_FEEDBACK_POL_);

        return Errors::NO_ERROR;
    }
}