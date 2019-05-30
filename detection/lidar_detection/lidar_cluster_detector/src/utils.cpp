/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-04-29 16:27:16
 * @LastEditTime: 2019-04-29 19:37:59
 */

#include "utils.h"

namespace LidarDetector {
/**
 * @description: opencv2 的功能函数 产生随机颜色数组 
 */
void generateColors(std::vector<cv::Scalar>& colors, size_t count, size_t factor)
{
    if (count < 1)
        return;

    colors.resize(count);

    if (count == 1) {
        colors[0] = cv::Scalar(0, 0, 255); // red
        return;
    }
    if (count == 2) {
        colors[0] = cv::Scalar(0, 0, 255); // red
        colors[1] = cv::Scalar(0, 255, 0); // green
        return;
    }

    // Generate a set of colors in RGB space. A size of the set is severel times (=factor) larger then
    // the needed count of colors.
    cv::Mat bgr(1, (int)(count * factor), CV_8UC3);
    randu(bgr, 0, 256);

    // Convert the colors set to Lab space.
    // Distances between colors in this space correspond a human perception.
    cv::Mat lab;
    cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    // Subsample colors from the generated set so that
    // to maximize the minimum distances between each other.
    // Douglas-Peucker algorithm is used for this.
    cv::Mat lab_subset;
    downsamplePoints(lab, lab_subset, count);

    // Convert subsampled colors back to RGB
    cv::Mat bgr_subset;
    cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);

    CV_Assert(bgr_subset.total() == count);
    for (size_t i = 0; i < count; i++) {
        cv::Point3_<uchar> c = bgr_subset.at<cv::Point3_<uchar>>((int)i);
        colors[i] = cv::Scalar(c.x, c.y, c.z);
    }
}

void downsamplePoints(const cv::Mat& src, cv::Mat& dst, size_t count)
{
    CV_Assert(count >= 2);
    CV_Assert(src.cols == 1 || src.rows == 1);
    CV_Assert(src.total() >= count);
    CV_Assert(src.type() == CV_8UC3);

    dst.create(1, (int)count, CV_8UC3);
    //TODO: optimize by exploiting symmetry in the distance matrix
    cv::Mat dists((int)src.total(), (int)src.total(), CV_32FC1, cv::Scalar(0));
    if (dists.empty())
        std::cerr << "Such big matrix cann't be created." << std::endl;

    for (int i = 0; i < dists.rows; i++) {
        for (int j = i; j < dists.cols; j++) {
            float dist = (float)norm(src.at<cv::Point3_<uchar>>(i) - src.at<cv::Point3_<uchar>>(j));
            dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
        }
    }

    double maxVal;
    cv::Point maxLoc;
    minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

    dst.at<cv::Point3_<uchar>>(0) = src.at<cv::Point3_<uchar>>(maxLoc.x);
    dst.at<cv::Point3_<uchar>>(1) = src.at<cv::Point3_<uchar>>(maxLoc.y);

    cv::Mat activedDists(0, dists.cols, dists.type());
    cv::Mat candidatePointsMask(1, dists.cols, CV_8UC1, cv::Scalar(255));
    activedDists.push_back(dists.row(maxLoc.y));
    candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

    for (size_t i = 2; i < count; i++) {
        activedDists.push_back(dists.row(maxLoc.x));
        candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

        cv::Mat minDists;
        reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
        minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
        dst.at<cv::Point3_<uchar>>((int)i) = src.at<cv::Point3_<uchar>>(maxLoc.x);
    }
}

void splitString(const std::string& in_string, std::vector<double>& out_array)
{
    std::string tmp;
    std::istringstream in(in_string);
    while (std::getline(in, tmp, ',')) {
        out_array.push_back(stod(tmp));
    }
}
}