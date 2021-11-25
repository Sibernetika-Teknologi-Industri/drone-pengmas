/**
 * Class for image processing
 * Based on cv_bridge and opencv
 */

#include <image_processor/ImageProcessor.hpp>

namespace img_proc
{
    ImageProcessor::ImageProcessor() :
        it_(nh_),
        useUMat_(true),
        windowName_("ImageProcessor"),
        imgProcSrv_(nh_.advertiseService("img_proc_server", &ImageProcessor::serviceCB, this)) // advertise service server
    {
        ROS_INFO("Image processing server initializing");
        
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

        // Get video stream namespace
        if (nh_.getParam("/vid_ns", vidNs_) && vidNs_ != "")
        {
            ROS_DEBUG_STREAM("Video stream namespace: " << vidNs_);
            vidNs_ = "/" + vidNs_;
        }
        else
        {
            ROS_WARN("Video stream namespace not found. Setting to /camera");
            
            vidNs_ = "/camera";
        }

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

        // Get param for showing video stream or not
        if (nh_.getParam("/show_vid_stream", showVidStream_))
        {
            if (showVidStream_)
            {
                // Window to show video stream
                cv::namedWindow(windowName_);

                ROS_INFO("Will show video stream");
            }
            else
            {
                ROS_INFO("Will not show video stream");
            }
        }
        else
        {
            ROS_WARN("show_vid_stream param not found. Will use default value (true).");
            ROS_WARN("Will show video stream.");

            showVidStream_ = true;
        }

        // Subscribe to input video feed
        std::string imgTopic(vidNs_ + "/image_raw");
        ROS_DEBUG_STREAM("Subscribing to: " << imgTopic);
        imgSub_ = it_.subscribe(imgTopic, 1, &ImageProcessor::imgCB, this);

        ROS_INFO_STREAM("Image processor server initialized at: " << ros::this_node::getName());
    }

    ImageProcessor::~ImageProcessor()
    {
        if (showVidStream_)
        {
            cv::destroyAllWindows();
        }
    }

    // ----- PRIVATE METHODS -----

    // ----- CALLBACK FUNCTIONS -----

    /**
     * Callback function to store image from video stream
     * Will be invoked for each new message from video stream
     */
    void ImageProcessor::imgCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cvPtr;

        // Check if video stream is in colour or monochrome
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        else
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        
        // Check which image is available for use
        // Prioritize UMat over Mat
        useUMat_ = true;
        cvPtr->image.copyTo(imgGPU_);
        if (imgGPU_.empty())
        {
            useUMat_ = false;
            cvPtr->image.copyTo(img_);
        }

        // If toggled, show video stream in window
        if (showVidStream_)
        {
            if (useUMat_)
            {
                cv::imshow(windowName_, imgGPU_);
            }
            else
            {
                cv::imshow(windowName_, img_);
            }

            cv::waitKey(1);
        }
    }

    /**
     * Function to process request and response message
     * Will be invoked for each service call
     */
    bool ImageProcessor::serviceCB(robot_msgs::ProcessImage::Request& req,
                                   robot_msgs::ProcessImage::Response& res)
    {
        // Info on which funciton to be run
        uint8_t funcID {req.func_id};

        // Run image processing function
        uint8_t funcRetVal;
        switch (funcID)
        {
            case robot_msgs::ImgProcMap::SAVE_IMG:
                ROS_INFO("Running: saveImg");
                funcRetVal = saveImg(req.params);
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
            ROS_ERROR("Image processing function invoke failed");

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

    // ----- IMAGE PROCESSING FUNCTIONS -----

    /**
     * Function to save image from video stream
     * Returns true if there is no error, and otherwise
     * 
     * Params: - strings[0] = full save location
     *         - strings[1] = image file name, with file extension
     */
    uint8_t ImageProcessor::saveImg(const Params& params)
    {
        // Check params
        if (params.strings.size() < 2)
        {
            ROS_ERROR("Number of strings params passed to saveImg too little");

            return Errors::INVALID_ARGUMENT;
        }

        // Store params data
        const std::string saveLoc(params.strings[0]);
        const std::string fileName(params.strings[1]);

        ROS_INFO_STREAM("Saving latest image from video stream into " <<
                        saveLoc + fileName);

        // Check if storage folder exists
        // If the folder doesn't exist, create a new folder
        if (boost::filesystem::exists(saveLoc) && boost::filesystem::is_directory(saveLoc)) // folder exists
        {
            ROS_DEBUG_STREAM(saveLoc << " exists");
        }
        else // folder doesn't exist
        {
            ROS_DEBUG_STREAM(saveLoc << " doesn't exist. Will create a new folder with the same name");

            boost::filesystem::create_directory(saveLoc);
        }
        
        // Use UMat if available
        if (useUMat_)
        {
            ROS_DEBUG("Saving image from UMat");

            // Wait until image is available
            while (imgGPU_.empty())
            {
                ;
            }

            // Matrix to save image
            cv::UMat saveImgGPU;

            // Convert to 32 bit image
            imgGPU_.convertTo(saveImgGPU, CV_32F);

            // Save image
            if (cv::imwrite(params.strings[0] + params.strings[1], saveImgGPU))
            {
                ROS_INFO("Image saved successfully");

                return Errors::NO_ERROR;
            }
            else
            {
                ROS_ERROR("Image saving failed");

                return Errors::FUNCTION_INVOKE_FAILED;
            }
        }
        else
        {
            ROS_DEBUG("Saving image from Mat");

            // Wait until image is available
            while (img_.empty())
            {
                ;
            }

            // Matrix to save image
            cv::Mat saveImg;
            
            // Convert to 32 bit image
            img_.convertTo(saveImg, CV_32F);

            // Save image
            if (cv::imwrite(params.strings[0] + params.strings[1], saveImg))
            {
                ROS_INFO("Image saved successfully");

                return Errors::NO_ERROR;
            }
            else
            {
                ROS_ERROR("Image saving failed");

                return Errors::FUNCTION_INVOKE_FAILED;
            }
        }

        // If the procedure reached this point, it means something's wrong
        return Errors::FUNCTION_INVOKE_FAILED;
    }

    // ----- PUBLIC METHODS -----
}