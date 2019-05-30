/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-05 20:42:26
 * @LastEditTime: 2019-05-01 22:41:00
 */
#ifndef CLUSTER_H
#define CLUSTER_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <ros/ros.h>

namespace LidarDetector {

class Cluster {
private:
    pcl::PointCloud<pcl::PointXYZRGB> pc;

    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::PointXYZ central_point;

    float length, width, height;
    std::string label;
    std::vector<pcl::PointXYZ> ploygon_points;

public:
    Cluster();
    ~Cluster();
    void SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
        std::vector<cv::Scalar>& color_table,
        const std::vector<int>& cluster_indices, const double& cluster_id);
    pcl::PointXYZ GetCentroid();
    pcl::PointCloud<pcl::PointXYZRGB> GetCloud();
    float GetLength();
    float GetWidth();
    float GetHeight();
};

typedef boost::shared_ptr<Cluster> ClusterPtr;

void generateColors(std::vector<cv::Scalar>& colors, size_t count, size_t factor);

void downsamplePoints(const cv::Mat& src, cv::Mat& dst, size_t count);
}

#endif //CLUSTER_H
