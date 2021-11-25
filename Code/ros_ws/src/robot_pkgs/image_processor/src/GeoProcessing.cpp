/**
 * Class for image geoprocessing
 * Modified from https://www.exiv2.org/doc/geotag_8cpp-example.html
 */

#include <image_processor/GeoProcessing.hpp>

namespace img_proc
{
    GeoProcessing::GeoProcessing() :
        time_(0),
        lon_(0),
        lat_(0),
        alt_(0),
        delta_(0),
        geoPosSrv_(nh_.advertiseService("geo_pos_server", &GeoProcessing::serviceCB, this)) // advertise service
    {
        ROS_INFO("Geo position server initializing");

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

        // Subscribe to GPS data topic
        ROS_DEBUG("Subscribing to GPS data");
        GPSSub_ = nh_.subscribe<sensor_msgs::NavSatFix>(mavrosNs_ + "/mavros/global_position/global", 1, &GeoProcessing::GPSCB, this);
        ROS_DEBUG("Subscribed to %s/mavros/global_position/global", mavrosNs_.c_str());

        ROS_INFO_STREAM("Geo position server initialized at: " << ros::this_node::getName());
    }

    // ----- PRIVATE METHODS -----

    // ----- CALLBACK FUNCTIONS -----

    /**
     * Function to get GPS data
     * Will be invoked every time GPS data message is received
     */
    void GeoProcessing::GPSCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        lon_ = msg->longitude;
        lat_ = msg->latitude;
        alt_ = msg->altitude;
    }

    /**
     * Function to process request and response message
     * Will be invoked for each service call
     */
    bool GeoProcessing::serviceCB(robot_msgs::GeoProcessing::Request& req,
                                robot_msgs::GeoProcessing::Response& res)
    {
        // Info on which funciton to be run
        uint8_t funcID {req.func_id};

        // Run image processing function
        int funcRetVal;
        switch (funcID)
        {
            case robot_msgs::GeoProcMap::GEOTAG:
                ROS_INFO("Running: geotag");
                funcRetVal = geotag(req.params);
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

    // ----- UTILITY FUNCTIONS -----

    /**
     * Function to read time data from image
     * 
     * path: full path to image file
     * pS: idk
     */
    time_t GeoProcessing::readImageTime(const std::string& path, std::string* pS)
    {
        time_t result{0};

        const char* dateStrings[]   {"Exif.Photo.DateTimeOriginal", 
                                     "Exif.Photo.DateTimeDigitized", 
                                     "Exif.Image.DateTime"};
        const char* dateString      {dateStrings[0]};

        int8_t seq{0};
        do 
        {
            // Store dateString data
            dateString = dateStrings[seq++];

            // Open image
            Exiv2::Image::UniquePtr image {Exiv2::ImageFactory::open(path)};

            // Will only continue if image is opened
            if ( image.get() ) 
            {
                // Read and save image metadata
                image->readMetadata();
                Exiv2::ExifData &exifData {image->exifData()};

                ROS_DEBUG("%s => %s\n",
                          dateString, exifData[dateString].toString().c_str());

                // Parse time data
                result = parseTime(exifData[dateString].toString().c_str(), true);
                
                if ( result && pS ) 
                {
                    *pS = exifData[dateString].toString();
                }
            }
        } while ( !result && dateString && seq <= 2 );

        return result ;
    }

    /**
     * Function to parse time values from exif string
     * 
     * arg: time in exif string
     * bAdjust: idk
     */
    time_t GeoProcessing::parseTime(const char* arg, bool bAdjust)
    {
        time_t result = 0 ;

        // Example data

        // 559 rmills@rmills-imac:~/bin $ exiv2 -pa ~/R.jpg | grep -i date
        // Exif.Image.DateTime                          Ascii      20  2009:08:03 08:58:57
        // Exif.Photo.DateTimeOriginal                  Ascii      20  2009:08:03 08:58:57
        // Exif.Photo.DateTimeDigitized                 Ascii      20  2009:08:03 08:58:57
        // Exif.GPSInfo.GPSDateStamp                    Ascii      21  2009-08-03T15:58:57Z

        // <time>2012-07-14T17:33:16Z</time>

        if ( strstr(arg, ":") || strstr(arg, "-") ) 
        {
            // Buffers
            int YY  {0}; // year
            int MM  {0}; // month
            int DD  {0}; // date
            int HH  {0}; // hour
            int mm  {0}; // minute
            int SS1 {0}; // second
            char a  {0}; // first -
            char b  {0}; // second -
            char c  {0}; // T
            char d  {0}; // first :
            char e  {0}; // second :
            struct tm T; // for containing time data

            // Store date stamp and time stamp data
            sscanf(arg,
                "%d%c%d%c%d%c%d%c%d%c%d",
                &YY, &a, &MM, &b, &DD, &c, &HH, &d, &mm, &e, &SS1);

            // Empty out T
            memset(&T, 0, sizeof(T));

            // Fill T
            T.tm_min  = mm;
            T.tm_hour = HH;
            T.tm_sec  = SS1;
            if ( bAdjust ) 
            {
                T.tm_sec -= Adjust();
            }
            T.tm_year = YY - 1900;
            T.tm_mon  = MM - 1;
            T.tm_mday = DD;
            T.tm_isdst = -1; // determine value automatically (otherwise hour may shift)

            result = mktime(&T);
        }

        return result ;
    }

    // ----- GeoProcessing FUNCTIONS -----

    /**
     * Function to geotag image
     * Will return true if there's no error, and otherwise
     * Params: - strings[0] = full save folder location
     *         - strings[1] = image file name, with file extension
     *         - strings[2] = CSV file name, with file extension
     */
    uint8_t GeoProcessing::geotag(const Params& params)
    {
        // Check params
        if (params.strings.size() < 3)
        {
            ROS_ERROR("Number of strings param passed into geotag too little");

            return Errors::INVALID_ARGUMENT;
        }
        
        // Storing params data
        const std::string folderLoc (params.strings[0]); // folder location
        const std::string imgName   (params.strings[1]); // image name
        const std::string CSVName   (params.strings[2]); // CSV file name

        ROS_DEBUG("Geotagging %s and saving GPS data to %s", 
                  (folderLoc + imgName).c_str(), (folderLoc + CSVName).c_str());

        // For storing time stamp
        std::string stamp;

        // Time in long format
        time_t t                        {readImageTime(folderLoc + imgName, &stamp)};

        // Open image
        Exiv2::Image::UniquePtr image   {Exiv2::ImageFactory::open(folderLoc + imgName)};

        // Will only continue if image is opened successfully
        if ( image.get() ) 
        {
            // Read image metadata
            image->readMetadata();
            Exiv2::ExifData& exifData {image->exifData()};

            // Store Exif data in buffer
            struct Data
            {
                std::string ImageName;

                std::string GPSProcessingMethod;
                std::string GPSVersionID;
                std::string GPSMapDatum;

                std::string GPSLatitude;
                std::string GPSLongitude;
                std::string GPSAltitude;

                std::string GPSAltitudeRef;
                std::string GPSLatitudeRef;
                std::string GPSLongitudeRef;

                std::string GPSDateStamp;
                std::string GPSTimeStamp;
                int GPSTag;
            };
            Data data;

            data.ImageName              = imgName;

            data.GPSProcessingMethod    = "charset=Ascii HYBRID-FIX";
            data.GPSVersionID           = "2 2 0 0";
            data.GPSMapDatum            = "WGS-84";

            data.GPSLatitude            = toExifString(lat_, true, true);
            data.GPSLongitude           = toExifString(lon_, true, false);
            data.GPSAltitude            = toExifString(alt_);

            data.GPSAltitudeRef         = (alt_ < 0.0f) ? "1" : "0";
            data.GPSLatitudeRef         = (lat_ > 0.0f) ? "N" : "S";
            data.GPSLongitudeRef        = (lon_ > 0.0f) ? "E" : "W";

            data.GPSDateStamp           = stamp;
            data.GPSTimeStamp           = toExifTimeStamp(stamp);
            data.GPSTag                 = 4908;

            // Modify exif data
            exifData["Exif.GPSInfo.GPSProcessingMethod" ] = data.GPSProcessingMethod;
            exifData["Exif.GPSInfo.GPSVersionID"        ] = data.GPSVersionID;
            exifData["Exif.GPSInfo.GPSMapDatum"         ] = data.GPSMapDatum;

            exifData["Exif.GPSInfo.GPSLatitude"         ] = data.GPSLatitude;
            exifData["Exif.GPSInfo.GPSLongitude"        ] = data.GPSLongitude;
            exifData["Exif.GPSInfo.GPSAltitude"         ] = data.GPSAltitude;

            exifData["Exif.GPSInfo.GPSAltitudeRef"      ] = data.GPSAltitudeRef;
            exifData["Exif.GPSInfo.GPSLatitudeRef"      ] = data.GPSLatitudeRef;
            exifData["Exif.GPSInfo.GPSLongitudeRef"     ] = data.GPSLongitudeRef;

            exifData["Exif.GPSInfo.GPSDateStamp"        ] = data.GPSDateStamp;
            exifData["Exif.GPSInfo.GPSTimeStamp"        ] = data.GPSTimeStamp;
            exifData["Exif.Image.GPSTag"                ] = data.GPSTag;

            ROS_DEBUG("%s geotag successful", (folderLoc + imgName).c_str());

            // Store GPS data inside .csv file
            // If .csv file doesn't exist, create a new one
              
            // Object to write to CSV
            CSVWriter csv(",");

            // Check file existence
            if (boost::filesystem::exists(folderLoc + CSVName) && boost::filesystem::is_regular_file(folderLoc + CSVName)) // .csv file exists
            {
                ROS_DEBUG_STREAM(folderLoc + CSVName << " exists");
            }
            else // .csv file doesn't exist
            {
                ROS_DEBUG_STREAM(folderLoc + CSVName << " doesn't exist. Will create a new file with the same name");

                // Save header names
                csv.newRow() << "ImageName"

                                << "GPSProcessingMethod" 
                                << "GPSVersionID"
                                << "GPSMapDatum"

                                << "GPSLatitude"
                                << "GPSLongitude"
                                << "GPSAltitude"

                                << "GPSAltitudeRef"
                                << "GPSLatitudeRef"
                                << "GPSLongitudeRef"

                                << "GPSDateStamp"
                                << "GPSTimeStamp"
                                << "GPSTag";
            }

            // Save data
            csv.newRow() << data.ImageName

                         << data.GPSProcessingMethod
                         << data.GPSVersionID
                         << data.GPSMapDatum

                         << data.GPSLatitude
                         << data.GPSLongitude
                         << data.GPSAltitude
                            
                         << data.GPSAltitudeRef
                         << data.GPSLatitudeRef
                         << data.GPSLongitudeRef
                            
                         << data.GPSDateStamp
                         << data.GPSTimeStamp
                         << data.GPSTag;

            // Write to .csv file
            csv.writeToFile(folderLoc + CSVName, true);

            ROS_DEBUG("Saving GPS data to %s successful", (folderLoc + CSVName).c_str());
        }
        else
        {
            ROS_ERROR("Image can't be opened");

            return Errors::OTHERS;
        }

        // If this point is reached, it means there's no error
        return Errors::NO_ERROR;
    }

    // ----- PUBLIC METHODS -----

    /**
     * idk what's it for
     */
    int GeoProcessing::Adjust() 
    {
        return adjust_ + tz_ + dst_ ;
    }

    // ----- STATIC HELPER FUNCTIONS -----

    /**
     * Function to convert latitude and longitude degree value into exif string
     * 
     * d: degree, can be latitude or longitude
     * isRat: if true, will return result in rational number
     * isLat: must be true if the value passed is latitude
     */
    std::string GeoProcessing::toExifString(double d, bool isRat, bool isLat)
    {
        // Check location
        const char* NS      {(d >= 0) ? "N" : "S"};
        const char* EW      {(d >= 0.0) ? "E" : "W"};
        const char* NSEW    {isLat ? NS : EW};

        // Make d as absolute value
        abs(d);

        // Convert d's unit into degree, minute, and second
        int deg {static_cast<int>(d)};
        d -= deg;
        d *= 60;
        int min {static_cast<int>(d)};
        d -= min;
        d *= 60;
        int sec {static_cast<int>(d)};

        // Convert data into exif string
        char result[200];
        const char* gDeg {"deg"}; // using ASCII standard
        if (isRat) // in fraction
        {
            snprintf(result, 
                     sizeof(result), 
                     "%d/1 %d/1 %d/1" , 
                     deg, min, sec);
        }
        else // in decimals
        {
            snprintf(result,
                     sizeof(result),
                     "%03d%s%02d'%02d\"%s" ,
                     deg, gDeg, min, sec, NSEW);
        }

        return std::string(result);
    }
    
    /**
     * Overloaded function
     * For converting altitude value to exif string
     * 
     * d: altitude
     */
    std::string GeoProcessing::toExifString(double d)
    {
        // Buffers
        char result[200];
        d *= 100;

        // Store data
        snprintf(result, 
                 sizeof(result), 
                 "%d/100", 
                 abs(static_cast<int>(d)));

        return std::string(result);
    }
    
    /**
     * Function to convert time stamp into exif string format
     * Will only extract hour, minute, and second
     * 
     * t: time stamp
     */
    std::string GeoProcessing::toExifTimeStamp(std::string& t)
    {
        /**
         * Example data
         * <time>2012-07-14T17:33:16Z</time>
         */

        // Buffers
        char result[200];
        const char* arg {t.c_str()};
        int HH  {0}; // hour
        int mm  {0}; // minute
        int SS1 {0}; // second

        if ( strstr(arg,":") || strstr(arg,"-") ) 
        {
            int YY  {0}; // year
            int MM  {0}; // month
            int DD  {0}; // day
            char a  {0}; // first -
            char b  {0}; // second -
            char c  {0}; // T
            char d  {0}; // first :
            char e  {0}; // second :

            sscanf(arg,
                   "%d%c%d%c%d%c%d%c%d%c%d",
                   &YY, &a, &MM, &b, &DD, &c, &HH, &d, &mm, &e, &SS1);
        }

        // Store time stamp data
        snprintf(result,
                 sizeof(result),
                 "%d/1 %d/1 %d/1",
                 HH, mm, SS1);

        return std::string(result);
    }

    // Default value for static member variables
    int    GeoProcessing::adjust_     {0};
    int    GeoProcessing::tz_         {timeZoneAdjust()};
    int    GeoProcessing::dst_        {0};
    time_t GeoProcessing::deltaMax_   {60};

    /**
     * Function to adjust offsets caused by time zone
     * West of GMT is negative (PDT = Pacific Daylight = -07:00 == -25200 seconds
     */
    int timeZoneAdjust()
    {
        time_t now {time(nullptr)};
        int offset;

        #if   defined(_MSC_VER) || defined(__MINGW__)
            TIME_ZONE_INFORMATION TimeZoneInfo;
            GetTimeZoneInformation( &TimeZoneInfo );
            offset = - (((int)TimeZoneInfo.Bias + (int)TimeZoneInfo.DaylightBias) * 60);
            UNUSED(now);

        #elif defined(__CYGWIN__)
            struct tm lcopy = *localtime(&now);
            time_t    gmt   =  timegm(&lcopy) ; // timegm modifies lcopy
            offset          = (int) ( ((long signed int) gmt) - ((long signed int) now) ) ;
        
        #elif defined(OS_SOLARIS) || defined(__sun__)
            struct tm local = *localtime(&now) ;
            time_t local_tt = (int) mktime(&local);
            time_t time_gmt = (int) mktime(gmtime(&now));
            offset          = time_gmt - local_tt;
        
        #else
            struct tm local = *localtime(&now) ;
            offset          = local.tm_gmtoff ;

        #if EXIV2_DEBUG_MESSAGES
            struct tm utc = *gmtime(&now);
            printf("utc  :  offset = %6d dst = %d time = %s", 0     ,utc  .tm_isdst, asctime(&utc  ));
            printf("local:  offset = %6d dst = %d time = %s", offset,local.tm_isdst, asctime(&local));
            printf("timeZoneAdjust = %6d\n",offset);
        #endif
        #endif

        return offset ;
    }
}