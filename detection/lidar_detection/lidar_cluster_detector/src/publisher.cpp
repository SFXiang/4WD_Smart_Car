/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-04-29 16:22:31
 * @LastEditTime: 2019-05-01 22:50:00
 */

#include "lidar_euclidean_cluster.h"

namespace LidarDetector {
/**
 * @description: 点云发布函数 
 */
void LidarClusterDetector::pubPointCloud(
    const ros::Publisher& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pointcloud)
{
    sensor_msgs::PointCloud2 msg_pointcloud;
    pcl::toROSMsg(*in_pointcloud, msg_pointcloud);
    msg_pointcloud.header = msg_header;
    publisher.publish(msg_pointcloud);
}

void LidarClusterDetector::pubClustersRviz(std::vector<Cluster>& in_clusters,
    ros::Publisher& pubRviz)
{

    visualization_msgs::MarkerArray objs_marker;
    visualization_msgs::Marker obj_marker;

    obj_marker.header.frame_id = "velodyne";
    obj_marker.header.stamp = ros::Time();
    obj_marker.type = visualization_msgs::Marker::CUBE;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.color.a = 0.6;
    obj_marker.lifetime = ros::Duration(0.1);
    objs_marker.markers.clear();
    for (size_t k = 0; k < in_clusters.size(); k++) {

        // Rviz marker
        obj_marker.id = k;
        obj_marker.color.b = 0;
        obj_marker.color.g = 0;
        obj_marker.color.r = 1;
        obj_marker.pose.position.x = in_clusters[k].GetCentroid().x;
        obj_marker.pose.position.y = in_clusters[k].GetCentroid().y;
        obj_marker.pose.position.z = in_clusters[k].GetCentroid().z;
        obj_marker.scale.x = in_clusters[k].GetLength();
        obj_marker.scale.y = in_clusters[k].GetWidth();
        obj_marker.scale.z = in_clusters[k].GetHeight();
        objs_marker.markers.push_back(obj_marker);
    }
    pubRviz.publish(objs_marker);
}
}