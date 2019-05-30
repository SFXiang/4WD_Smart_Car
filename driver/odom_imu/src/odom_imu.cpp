#include "odom_imu/odom_imu.h"

OdomImu::~OdomImu()
{
}

bool OdomImu::init()
{
    ROS_INFO("Start init OdomImu.");

    first_imu_ = true;
    first_pose_ = true;
    tf_init_ = false;

    pnh_.param<double>("max_interval", param_max_interval_, 1.0);
    pnh_.param<double>("angle_vel_sensitive", param_angle_vel_sensitive_, 0.001);
    pnh_.param<double>("linear_vel_sensitive", param_linear_vel_sensitive_, 0.001);
    pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
    pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
    pnh_.param<std::string>("sub_imu_topic", param_imu_topic_, "/imu");

    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(param_imu_topic_, 500, boost::bind(&OdomImu::imuCB, this, _1));
    sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 10, boost::bind(&OdomImu::poseCB, this, _1));
    // sub_encoder_ = nh_.subscribe<geometry_msgs::TwistStamped>("/wheel_circles", 100, boost::bind(&OdomImu::encoderCB, this, _1));
    // sub_mode_stat_ = nh_.subscribe<std_msgs::Int32>("/mode_stat", 1, boost::bind(&OdomImu::modeStatCB, this, _1));
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom/imu", 10);
    pub_pose_odom_ = nh_.advertise<geometry_msgs::PoseStamped>("/odom/predict_pose", 10);
    // msg_mode_stat_.data = 1;

    ROS_INFO("End init OdomImu.");

    return true;
}

// void OdomImu::encoderCB(const geometry_msgs::TwistStamped::ConstPtr& msg)
// {
//     if (first_encoder_) {
//         first_encoder_ = false;
//         ROS_INFO("Received first encoder.");
//     }
//     cur_vel_ = *msg;
//     // if ((msg_mode_stat_.data < 0 && cur_vel_.twist.linear.x > 0) || (msg_mode_stat_.data > 0 && cur_vel_.twist.linear.x < 0)){
//     //   cur_vel_.twist.linear.x *= -1;
//     // }
// }

void OdomImu::poseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (first_pose_) {
        ROS_WARN_STREAM("odom_imu -> received current_pose");
    }
    static int waitNum = 10;
    static int cnt = 0;
    if (cnt < waitNum) {
        cnt++;
        return;
    }
    if (q.size() < 5) {
        q.push(*msg);
        return;
    }
    first_pose_ = false;
    q.pop();
    q.push(*msg);
    geometry_msgs::PoseStamped first = q.front();
    geometry_msgs::PoseStamped last = q.back();
    double dis = std::sqrt(std::pow(first.pose.position.x - last.pose.position.x, 2) + std::pow(first.pose.position.y - last.pose.position.y, 2));

    cur_vel_.header = msg->header;
    cur_vel_.twist.linear.x = dis / (last.header.stamp.toSec() - first.header.stamp.toSec());
    current_vel_y_ = 0;
    current_vel_z_ = 0;
    geometryPose2Pose(msg->pose, predict_pose_odom_);
    offset_odom_.reset();
}

void OdomImu::imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (first_imu_) {
        first_imu_ = false;
        pre_time_ = msg->header.stamp;
        return;
    } else if (first_pose_) {
        ROS_WARN("Have not received encoder, not publish odom");
        return;
    }

    double diff_time = (msg->header.stamp - pre_time_).toSec();
    if (diff_time > param_max_interval_) {
        ROS_WARN("Long time waiting for next imu msg. Igore this msg.");
        pre_time_ = msg->header.stamp;
        return;
    }
    // else if ((msg->header.stamp - cur_vel_.header.stamp).toSec() > param_max_interval_) {
    //     ROS_WARN("Long time waiting for encoder msg update. Igore this msg.");
    //     return;
    // }

    // 进行一个低通滤波
    double angle_vel_x = round(msg->angular_velocity.x / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    double angle_vel_y = round(msg->angular_velocity.y / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    double angle_vel_z = round(msg->angular_velocity.z / param_angle_vel_sensitive_) * param_angle_vel_sensitive_;
    double linear_acc_x = round(msg->linear_acceleration.x / param_linear_vel_sensitive_) * param_linear_vel_sensitive_;
    double linear_acc_y = round(msg->linear_acceleration.y / param_linear_vel_sensitive_) * param_linear_vel_sensitive_;
    double linear_acc_z = round(msg->linear_acceleration.z / param_linear_vel_sensitive_) * param_linear_vel_sensitive_;
    // linear_acc_y = 0.;
    // linear_acc_z = 0.;

    double offset_roll = angle_vel_x * diff_time;
    double offset_pitch = angle_vel_y * diff_time;
    double offset_yaw = angle_vel_z * diff_time;

    // current_pose_.roll += angle_vel_x * diff_time;
    // current_pose_.pitch += angle_vel_y * diff_time;
    current_pose_.yaw += angle_vel_z * diff_time;
    // current_pose_.roll = 0.;
    // current_pose_.pitch = 0.;

    double accX1 = linear_acc_x;
    double accY1 = std::cos(current_pose_.roll) * linear_acc_y - std::sin(current_pose_.roll) * linear_acc_z;
    double accZ1 = std::sin(current_pose_.roll) * linear_acc_y + std::sin(current_pose_.roll) * linear_acc_z;

    double accX2 = std::sin(current_pose_.pitch) * accZ1 + std::cos(current_pose_.pitch) * accX1;
    double accY2 = accY1;
    double accZ2 = std::cos(current_pose_.pitch) * accZ1 - std::cos(current_pose_.pitch) * accX1;

    double accX = std::cos(current_pose_.yaw) * accX2 - std::sin(current_pose_.yaw) * accY2;
    double accY = std::sin(current_pose_.yaw) * accX2 + std::cos(current_pose_.yaw) * accY2;
    double accZ = accZ2;

    // current_pose_.x += current_vel_x_ * diff_time + accX * diff_time * diff_time / 2.0;
    // current_pose_.y += current_vel_y_ * diff_time + accY * diff_time * diff_time / 2.0;
    // current_pose_.z += current_vel_z_ * diff_time + accZ * diff_time * diff_time / 2.0;
    current_pose_.x += cur_vel_.twist.linear.x * std::cos(current_pose_.yaw) * diff_time;
    current_pose_.y += cur_vel_.twist.linear.x * std::sin(current_pose_.yaw) * diff_time;
    current_pose_.z = 0.;

    current_vel_x_ += accX * diff_time;
    current_vel_y_ += accY * diff_time;
    current_vel_z_ += accZ * diff_time;

    pre_pose_ = current_pose_;
    pre_time_ = msg->header.stamp;

    if (!tf_init_) {
        try {
            tf_listener_.waitForTransform(param_base_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
            tf_listener_.lookupTransform(param_base_frame_, msg->header.frame_id, ros::Time(0), tf_btoi_);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Transform error in imuCB: %s", ex.what());
            std::cout << "child frame id: " << msg->header.frame_id << std::endl;
            return;
        }
    }

    tf::Quaternion tmp_q;
    tmp_q.setRPY(0., 0., current_pose_.yaw);

    tf::Transform transform2(tmp_q, tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
    // transform odom->imu to odom->base
    tf::Transform transform = transform2 * tf_btoi_.inverse();
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, param_odom_frame_, param_base_frame_));

    msg_odom_.header.stamp = msg->header.stamp;
    msg_odom_.header.frame_id = param_odom_frame_;
    msg_odom_.child_frame_id = param_base_frame_;
    tf::pointTFToMsg(transform.getOrigin(), msg_odom_.pose.pose.position);
    tf::quaternionTFToMsg(transform.getRotation(), msg_odom_.pose.pose.orientation);
    msg_odom_.twist.twist.angular.x = angle_vel_x;
    msg_odom_.twist.twist.angular.y = angle_vel_y;
    msg_odom_.twist.twist.angular.z = angle_vel_z;
    msg_odom_.twist.twist.linear.x = cur_vel_.twist.linear.x;
    msg_odom_.twist.twist.linear.y = cur_vel_.twist.linear.y;
    msg_odom_.twist.twist.linear.z = cur_vel_.twist.linear.z;
    pub_odom_.publish(msg_odom_);
    debug(msg_odom_);
}

void OdomImu::debug(const nav_msgs::Odometry msg)
{
    static bool odom_init_ = false;
    static ros::Time pre_odom_time_;
    if (!odom_init_) {
        odom_init_ = true;
        pre_odom_time_ = msg.header.stamp;
        ROS_INFO("Init odom.");
        return;
    }
    double diff_time = (msg.header.stamp - pre_odom_time_).toSec();
    if (diff_time > 1.0) {
        ROS_WARN("Long time(%f s) waiting for odom msg, ignore this msg.", diff_time);
        pre_odom_time_ = msg.header.stamp;
        return;
    }

    msg_odom_ = msg;
    // offset_odom_.roll += msg->twist.twist.angular.x * diff_time;
    // offset_odom_.pitch += msg->twist.twist.angular.y * diff_time;
    offset_odom_.yaw += msg.twist.twist.angular.z * diff_time;
    double diff_x = msg.twist.twist.linear.x * diff_time;
    offset_odom_.x += std::cos(-predict_pose_odom_.pitch) * std::cos(predict_pose_odom_.yaw) * diff_x;
    offset_odom_.y += std::cos(-predict_pose_odom_.pitch) * std::sin(predict_pose_odom_.yaw) * diff_x;
    offset_odom_.z += std::sin(-predict_pose_odom_.pitch) * diff_x;
    // current_pose_odom_ += offset_odom_;  // error
    predict_pose_odom_ = pre_pose_ + offset_odom_;
    // pre_pose_odom_ = current_pose_odom_;
    // ROS_INFO("offset_odom.y: %.2f, %f", offset_odom_.y, ros::Time::now().toSec());
    // ROS_INFO("Current odom pose: (%.2f, %.2f, %.2f; %.2f, %.2f, %.2f)", current_pose_odom_.x, current_pose_odom_.y, current_pose_odom_.z, current_pose_odom_.roll, current_pose_odom_.pitch, current_pose_odom_.yaw);
    pre_odom_time_ = msg.header.stamp;

    geometry_msgs::PoseStamped msg_pose_odom_;
    msg_pose_odom_.header = msg.header;
    msg_pose_odom_.header.frame_id = "odom";
    pose2GeometryPose(msg_pose_odom_.pose, predict_pose_odom_);
    pub_pose_odom_.publish(msg_pose_odom_);
}

// void OdomImu::modeStatCB(const std_msgs::Int32::ConstPtr &msg)
// {
//   ROS_INFO("OdomImu: mode changed. pre: %d, cur: %d", msg_mode_stat_.data, msg->data);
//   msg_mode_stat_.data = msg->data;
// }