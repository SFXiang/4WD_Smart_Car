/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-05 20:38:52
 * @LastEditTime: 2019-05-01 22:41:59
 */
#include "cluster.h"

using namespace cv;

namespace LidarDetector {

Cluster::Cluster()
{
}

Cluster::~Cluster() {}

/**
 * @description: 设置cluster的相关成员变量
 */
void Cluster::SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    std::vector<cv::Scalar>& color_table,
    const std::vector<int>& cluster_indices, const double& cluster_id)
{

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < cluster_indices.size(); i++) {
        pcl::PointXYZRGB p;

        p.x = in_cloud->points[i].x;

        p.y = in_cloud->points[i].y;
        p.z = in_cloud->points[i].z;
        p.r = (int)color_table[cluster_id].val[0];
        p.g = (int)color_table[cluster_id].val[1];
        p.b = (int)color_table[cluster_id].val[2];

        pc.points.push_back(p);

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;

        central_point.x += p.x;
        central_point.y += p.y;
        central_point.z += p.z;
    }

    if (cluster_indices.size() > 0) {
        central_point.x /= cluster_indices.size();
        central_point.y /= cluster_indices.size();
        central_point.z /= cluster_indices.size();
    }

    min_point.x = min_x;
    min_point.y = min_y;
    min_point.z = min_z;

    max_point.x = max_x;
    max_point.y = max_y;
    max_point.z = max_z;

    length = max_x - min_x;
    width = max_y - min_y;
    height = max_z - min_z;

    std::vector<cv::Point2f> points_2d;
    for (size_t j = 0; j < pc.points.size(); j++) {
        cv::Point2f pp;
        pp.x = pc.points[j].x;
        pp.y = pc.points[j].y;
        points_2d.push_back(pp);
    }
    std::vector<cv::Point2f> hull;

    cv::convexHull(points_2d, hull);
    for (size_t k = 0; k < hull.size(); k++) {
        pcl::PointXYZ pp;
        pp.x = hull[k].x;
        pp.y = hull[k].y;
        pp.z = 0;
        ploygon_points.push_back(pp);
    }
}

pcl::PointXYZ Cluster::GetCentroid()
{
    return central_point;
}

pcl::PointCloud<pcl::PointXYZRGB> Cluster::GetCloud()
{
    return pc;
}

float Cluster::GetLength()
{
    return length;
}

float Cluster::GetWidth()
{
    return width;
}

float Cluster::GetHeight()
{
    return height;
}
}