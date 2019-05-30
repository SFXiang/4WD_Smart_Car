#include <iostream>
#include <lidar_detection_core.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_detection");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    LIDAR_DETECTION::lidar_detect detect(nh, pnh);
    detect.init();

    // ros::spin();
    return 0;
}