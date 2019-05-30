/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-04-29 19:37:16
 * @LastEditTime: 2019-04-29 19:38:20
 */

#include "cluster.h"

namespace LidarDetector {
/**
 * @description: opencv2 的功能函数 产生随机颜色数组 
 */
void generateColors(std::vector<cv::Scalar>& colors, size_t count, size_t factor);

void downsamplePoints(const cv::Mat& src, cv::Mat& dst, size_t count);

void splitString(const std::string& in_string, std::vector<double>& out_array);
}