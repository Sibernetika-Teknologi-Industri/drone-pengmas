/**
 * File for running gpio node
 */

#include <gpio/gpio_node.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gpio_server");

    gpio::GPIO gpio;

    ros::spin();

    return 0;
}