#pragma once

#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <robot_msgs/ProcessImage.h>
#include <robot_msgs/Params.h>
#include <robot_msgs/ErrorCodes.h>
#include <robot_msgs/ImgProcMap.h>

namespace img_proc
{
    class ImageProcessor
    {
        private:
            // ----- ALIASES -----
            using Params = robot_msgs::Params;
            using Errors = robot_msgs::ErrorCodes;

            // ----- ROS_STUFFS -----

            ros::NodeHandle nh_;
            ros::ServiceServer imgProcSrv_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber imgSub_;

            std::string imgProcNs_;

            // ----- OPENCV STUFFS -----

            const std::string windowName_;
            std::string vidNs_;
            cv::Mat img_;
            cv::UMat imgGPU_;
            bool useUMat_;
            bool showVidStream_;

            // ----- CALLBACKS -----
            
            void imgCB(const sensor_msgs::ImageConstPtr& msg);
            bool serviceCB(robot_msgs::ProcessImage::Request& req,
                           robot_msgs::ProcessImage::Response& res);

            // ----- IMAGE PROCESSING FUNCTIONS -----

            uint8_t saveImg (const Params& params);
            
        public:
            ImageProcessor();
            ~ImageProcessor();
    };
}
