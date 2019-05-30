#ifndef __ODOM_IMU__
#define __ODOM_IMU__

#include "user_protocol.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class OdomImu {
public:
    OdomImu(const ros::NodeHandle nh, const ros::NodeHandle pnh)
        : nh_(nh)
        , pnh_(pnh)
    {
    }
    ~OdomImu();
    bool init();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_btoi_;
    bool tf_init_;

    ros::Publisher pub_odom_;
    nav_msgs::Odometry msg_odom_;

    ros::Subscriber sub_imu_;
    sensor_msgs::Imu msg_imu_;
    // ros::Subscriber sub_encoder_;
    geometry_msgs::TwistStamped msg_encoder_;
    ros::Subscriber sub_pose_;
    geometry_msgs::TwistStamped cur_vel_;
    ros::Subscriber sub_mode_stat_;
    std_msgs::Int32 msg_mode_stat_;

    ros::Time pre_time_;
    bool first_imu_;
    // bool first_encoder_;
    bool first_pose_;

    pose pre_pose_;
    pose current_pose_;
    double current_vel_x_;
    double current_vel_y_;
    double current_vel_z_;

    double param_max_interval_;
    double param_angle_vel_sensitive_;
    double param_linear_vel_sensitive_;
    std::string param_base_frame_;
    std::string param_odom_frame_;
    std::string param_imu_topic_;

    std::queue<geometry_msgs::PoseStamped> q;
    pose predict_pose_odom_;
    pose offset_odom_;
    ros::Publisher pub_pose_odom_;

    void imuCB(const sensor_msgs::Imu::ConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    // void encoderCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
    // void modeStatCB(const std_msgs::Int32::ConstPtr& msg);
    void debug(const nav_msgs::Odometry msg);
};

#endif