/**
 * This file is for launching geographic position service
 */

#include <image_processor/geo_pos_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "geo_pos_server");

    img_proc::GeoProcessing geopos;

    ros::spin();

    return 0;
}