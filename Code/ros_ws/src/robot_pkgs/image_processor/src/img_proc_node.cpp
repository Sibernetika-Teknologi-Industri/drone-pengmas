/**
 * This file is for launching process image service
 */

#include <image_processor/img_proc_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "img_proc_server");

    img_proc::ImageProcessor imgProc;

    ros::spin();

    return 0;
}