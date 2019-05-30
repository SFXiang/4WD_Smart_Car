/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:25:55
 * @LastEditTime: 2019-05-01 23:16:28
 */

#include "lidar_euclidean_cluster.h"
#include "utils.h"

namespace LidarDetector {
LidarClusterDetector::LidarClusterDetector()
    : private_nh("~")
    , processing_now(false)
{
    initROS();

    splitString(str_range, dis_range);
    splitString(str_seg_distances, seg_distances);

    generateColors(color_table, 255, 100);
}

LidarClusterDetector::~LidarClusterDetector() {}

/**
 * @description: ROS参数初始化 
 */
void LidarClusterDetector::initROS()
{
    sub_rawPointCloud = nh.subscribe("velodyne_points", 10, &LidarClusterDetector::getPointCloud_cb, this);
    pub_testPointCloud = nh.advertise<sensor_msgs::PointCloud2>("test_pointcloud", 10);
    pub2_testPointCloud = nh.advertise<sensor_msgs::PointCloud2>("test2_pointcloud", 10);
    pub_clusters_Rviz = nh.advertise<visualization_msgs::MarkerArray>("lidarClusterRviz", 10);

    private_nh.param<double>("nearDistance", nearDistance, 2.0);
    private_nh.param<double>("farDistance", farDistance, 30);
    private_nh.param<double>("downsampleLeafSize", leaf_size, 0.5);
    private_nh.param<double>("height_threshhold", height_threshhold, 4.0);
    private_nh.param<double>("floor_max_height", floor_max_height, 0.3);
    private_nh.param<double>("floor_max_angle", floor_max_angle, 0.2);
    private_nh.param<double>("small_scale", small_scale, 0.5);
    private_nh.param<double>("large_scale", large_scale, 2.0);
    private_nh.param<double>("angle_threshold", angle_threshold, 0.5);
    private_nh.param<double>("radial_divide_angle", radial_divide_angle, 0.5);
    private_nh.param<double>("concentric_divide_distance", concentric_divide_distance, 0.1);
    private_nh.param<double>("min_local_height_threshold", min_local_height_threshold, 0.05);
    private_nh.param<double>("sensor_height", sensor_height, 0.37);
    private_nh.param<double>("local_threshold_slope", local_threshold_slope, 5.0);
    private_nh.param<double>("general_threshold_slope", general_threshold_slope, 3.0);
    private_nh.param<double>("left_right_dis_threshold", left_right_dis_threshold, 6.5);

    private_nh.param<std::string>("str_range", str_range, "15,30,45,60");
    private_nh.param<std::string>("str_seg_distances", str_seg_distances, "0.2,0.5,1.0,1.5,2.0");

    private_nh.param<double>("cluster_min_points", cluster_min_points, 10);
    private_nh.param<double>("cluster_max_points", cluster_max_points, 1000);
    private_nh.param<double>("cluster_merge_threshold", cluster_merge_threshold, 1.5);
}

/**
 * @description: 原始点云回调函数
 */
void LidarClusterDetector::getPointCloud_cb(
    const sensor_msgs::PointCloud2ConstPtr& msg_rawPointCloud)
{
    if (!processing_now) {
        processing_now = true;
        start_time = std::chrono::system_clock::now();

        msg_header = msg_rawPointCloud->header;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_sensor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr only_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr don_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ray_no_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ray_only_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg_rawPointCloud, *raw_sensor_cloud_ptr);

        clipCloud(raw_sensor_cloud_ptr, clipped_cloud_ptr, height_threshhold, nearDistance, farDistance, left_right_dis_threshold);
        // 也可以不用进行下采样
        downsampleCloud(clipped_cloud_ptr, downsample_cloud_ptr, leaf_size);

        removeFloorRayFiltered(downsample_cloud_ptr, ray_only_floor_cloud_ptr, ray_no_floor_cloud_ptr, sensor_height,
            local_threshold_slope, general_threshold_slope);

        segmentByDistance(raw_sensor_cloud_ptr, clustered_cloud_ptr);

        pubPointCloud(pub_testPointCloud, ray_only_floor_cloud_ptr);
        pubPointCloud(pub2_testPointCloud, ray_no_floor_cloud_ptr);

        processing_now = false;
    }
}

/**
 * @description: 基于距离的点云聚类 
 */
void LidarClusterDetector::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    // 根据距离不同，设置不同的聚类阈值
    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_segments_array[i] = tmp;
    }

    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++) {
        pcl::PointXYZ p;
        p.x = in_cloud_ptr->points[i].x;
        p.y = in_cloud_ptr->points[i].y;
        p.z = in_cloud_ptr->points[i].z;

        float origin_dis = sqrt(pow(p.x, 2) + pow(p.y, 2));

        if (origin_dis < dis_range[0]) {
            cloud_segments_array[0]->points.push_back(p);
        } else if (origin_dis < dis_range[1]) {
            cloud_segments_array[1]->points.push_back(p);
        } else if (origin_dis < dis_range[2]) {
            cloud_segments_array[2]->points.push_back(p);
        } else if (origin_dis < dis_range[3]) {
            cloud_segments_array[3]->points.push_back(p);
        } else {
            cloud_segments_array[4]->points.push_back(p);
        }
    }

    std::vector<Cluster> clusters;
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
        if (cloud_segments_array[i]->points.size() > 10) {
            clusterCpu(cloud_segments_array[i], clusters, seg_distances[i]);
        }
    }

    std::vector<Cluster> all_clusters;
    all_clusters.insert(all_clusters.end(), clusters.begin(), clusters.end());

    std::vector<Cluster> mid_clusters;
    std::vector<Cluster> final_clusters;
    if (all_clusters.size() > 0)
        checkAllForMerge(all_clusters, mid_clusters, cluster_merge_threshold);
    else
        mid_clusters = all_clusters;

    if (mid_clusters.size() > 0)
        checkAllForMerge(mid_clusters, final_clusters, cluster_merge_threshold);
    else
        final_clusters = mid_clusters;

    pubClustersRviz(final_clusters, pub_clusters_Rviz);
}

void LidarClusterDetector::checkClusterMerge(size_t in_cluster_id, std::vector<Cluster>& in_clusters,
    std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
    double in_merge_threshold)
{
    // std::cout << "checkClusterMerge" << std::endl;
    pcl::PointXYZ point_a = in_clusters[in_cluster_id].GetCentroid();
    for (size_t i = 0; i < in_clusters.size(); i++) {
        if (i != in_cluster_id && !in_out_visited_clusters[i]) {
            pcl::PointXYZ point_b = in_clusters[i].GetCentroid();
            double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
            if (distance <= in_merge_threshold) {
                in_out_visited_clusters[i] = true;
                out_merge_indices.push_back(i);
                // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
                checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
            }
        }
    }
}

void LidarClusterDetector::mergeClusters(std::vector<Cluster>& in_clusters, std::vector<Cluster>& out_clusters,
    std::vector<size_t> in_merge_indices, const size_t& current_index,
    std::vector<bool>& in_out_merged_clusters)
{
    // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
    pcl::PointCloud<pcl::PointXYZ> mono_cloud;
    Cluster merged_cluster;
    for (size_t i = 0; i < in_merge_indices.size(); i++) {
        sum_cloud += in_clusters[in_merge_indices[i]].GetCloud();
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    std::vector<int> indices(sum_cloud.points.size(), 0);
    for (size_t i = 0; i < sum_cloud.points.size(); i++) {
        indices[i] = i;
    }

    if (sum_cloud.points.size() > 0) {
        pcl::copyPointCloud(sum_cloud, mono_cloud);
        merged_cluster.SetCloud(mono_cloud.makeShared(), color_table, indices, current_index);
        out_clusters.push_back(merged_cluster);
    }
}

void LidarClusterDetector::checkAllForMerge(std::vector<Cluster>& in_clusters, std::vector<Cluster>& out_clusters,
    float in_merge_threshold)
{
    // std::cout << "checkAllForMerge" << std::endl;
    std::vector<bool> visited_clusters(in_clusters.size(), false);
    std::vector<bool> merged_clusters(in_clusters.size(), false);
    size_t current_index = 0;
    for (size_t i = 0; i < in_clusters.size(); i++) {
        if (!visited_clusters[i]) {
            visited_clusters[i] = true;
            std::vector<size_t> merge_indices;
            checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
            mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
        }
    }
    for (size_t i = 0; i < in_clusters.size(); i++) {
        // check for clusters not merged, add them to the output
        if (!merged_clusters[i]) {
            out_clusters.push_back(in_clusters[i]);
        }
    }

    // ClusterPtr cluster(new Cluster());
}

/**
 * @description: 对聚类结果进行后处理，生成聚类的相关信息保存到cluster类中
 */
void LidarClusterDetector::clusterCpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    std::vector<Cluster>& clusters, const double& max_cluster_dis)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*in_cloud, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++) {
        cloud_2d->points[i].z = 0;
    }

    tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc;
    // setClusterTolerance(). If you take a very small value, it can happen that
    // an actual object can be seen as multiple clusters. On the other hand, if
    // you set the value too high, it could happen, that multiple objects are
    // seen as one cluster. So our recommendation is to just test and try out
    // which value suits your dataset.
    euc.setClusterTolerance(max_cluster_dis);
    euc.setMinClusterSize(cluster_min_points);
    euc.setMaxClusterSize(cluster_max_points);
    euc.setSearchMethod(tree);
    euc.setInputCloud(cloud_2d);
    euc.extract(cluster_indices);

    for (size_t j = 0; j < cluster_indices.size(); j++) {
        Cluster one_cluster;
        one_cluster.SetCloud(in_cloud, color_table, cluster_indices[j].indices, j);

        clusters.push_back(one_cluster);
    }
}

/**
 * @description: 将原始点云转化为 XYZRadialTheta 结构的点云 
 */
void LidarClusterDetector::convertXYZ2XYZRT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    std::vector<PointCloudXYZRT>& out_radial_divided_cloud)
{
    out_radial_divided_cloud.clear();
    double radial_divide_num = ceil(360
        / radial_divide_angle);
    out_radial_divided_cloud.resize(radial_divide_num);

    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        PointXYZRT p;
        float radius = (float)sqrt(in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        float thera = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;

        if (thera < 0)
            thera += 360;

        size_t radial_div = (size_t)floor(thera / radial_divide_angle);
        size_t concentric_div = (size_t)floor(radius / concentric_divide_distance);

        p.point = in_cloud->points[i];
        p.radius = radius;
        p.theta = thera;
        p.radial_div = radial_div;
        p.concentric_div = concentric_div;
        p.original_index = i;

        out_radial_divided_cloud[radial_div].push_back(p);
    }

#pragma omp for
    for (size_t j = 0; j < out_radial_divided_cloud.size(); j++) {
        std::sort(out_radial_divided_cloud[j].begin(), out_radial_divided_cloud[j].end(),
            [](const PointXYZRT& a, const PointXYZRT& b) { return a.radius < b.radius; });
    }
}

} // namespace LidarDetector