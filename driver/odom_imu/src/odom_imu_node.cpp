#include "odom_imu/odom_imu.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_imu_node");
    ROS_INFO("Start odom_imu_node.\nMake sure the car is in stationary on the ground before starting this node, and the imu is calibrated.");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // bool use_imu_only_;
    // pnh.param<bool>("use_imu_only_", use_imu_only_, "false");
    // if (use_imu_only_) {
    //     ROS_WARN_STREAM("odom_imu -> Use imu only");
    //     IMU odom_imu(nh, pnh);
    //     if (!odom_imu.init()) {
    //         ROS_ERROR("Cannot init OdomImu!");
    //         exit(-1);
    //     }
    //     ros::spin();
    // } else {
    //     ROS_WARN_STREAM("odom_imu -> Use odom imu");
    //     OdomImu odom_imu(nh, pnh);
    //     if (!odom_imu.init()) {
    //         ROS_ERROR("Cannot init OdomImu!");
    //         exit(-1);
    //     }
    //     ros::spin();
    // }

    OdomImu odom_imu(nh, pnh);
    if (!odom_imu.init()) {
        ROS_ERROR("Cannot init OdomImu!");
        exit(-1);
    }

    ros::spin();

    return 0;
}