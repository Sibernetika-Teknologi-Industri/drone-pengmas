#pragma once

#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include <exiv2/exiv2.hpp>

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>

#include <robot_msgs/Params.h>
#include <robot_msgs/ErrorCodes.h>
#include <robot_msgs/GeoProcMap.h>
#include <robot_msgs/GeoProcessing.h>

#include <image_processor/CSVWriter.h>

#if defined(__MINGW32__) || defined(__MINGW64__)
# ifndef  __MINGW__
#  define __MINGW__
# endif
#endif

#ifndef  lengthof
#define  lengthof(x) (sizeof(*x)/sizeof(x))
#endif

#if defined(_MSC_VER) || defined(__MINGW__)
#include <windows.h>
char*    realpath(const char* file,char* path);
#define  lstat _stat
#define  stat  _stat
#if      _MSC_VER < 1400
#define strcpy_s(d,l,s) strcpy(d,s)
#define strcat_s(d,l,s) strcat(d,s)
#endif
#endif

#if ! defined(_MSC_VER)
#include <dirent.h>
#include <unistd.h>
#include <sys/param.h>
#define  stricmp strcasecmp
#endif

#ifndef _MAX_PATH
#define _MAX_PATH 1024
#endif

namespace img_proc
{
    class GeoProcessing
    {
        private:
            // ----- ALIASES -----

            using TimeDict_t = std::map<time_t, GeoProcessing>;
            using TimeDict_i = std::map<time_t, GeoProcessing>::iterator;
            using strings_t = std::vector<std::string>;
            using Params = robot_msgs::Params;
            using Errors = robot_msgs::ErrorCodes;

            // ----- ROS STUFFS -----

            ros::NodeHandle nh_;
            ros::Subscriber GPSSub_;
            ros::ServiceServer geoPosSrv_;

            // ----- DATA -----

            TimeDict_t  gTimeDict;
            strings_t   gFiles;
            time_t  time_;
            double  lon_;
            double  lat_;
            double  alt_;
            int     delta_;
            std::string times_;
            std::string mavrosNs_;

            // ----- CALLBACK FUNCTIONS -----
            
            void GPSCB      (const sensor_msgs::NavSatFix::ConstPtr& msg);
            bool serviceCB  (robot_msgs::GeoProcessing::Request& req,
                             robot_msgs::GeoProcessing::Response& res);

            // ----- UTILITY FUNCTIONS -----

            time_t readImageTime    (const std::string& path, std::string* pS = nullptr);
            time_t parseTime        (const char* , bool bAdjust=false);

            // ----- GeoProcessing FUNCTIONS -----

            uint8_t geotag(const Params& params);

        public:
            GeoProcessing();
            virtual ~GeoProcessing() = default;

            // ----- PUBLIC STATIC DATA -----

            static int adjust_;
            static int tz_;
            static int dst_;
            static time_t deltaMax_;

            // ----- PUBLIC STATIC MEMBER FUNCTIONS ------

            static int Adjust();
            static std::string toExifString     (double d, bool isRational, bool isLat);
            static std::string toExifString     (double d);
            static std::string toExifTimeStamp  (std::string& t);
    };

    int timeZoneAdjust(); // don't know where to put lol
}