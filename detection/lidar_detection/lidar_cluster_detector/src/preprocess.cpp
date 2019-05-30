/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-04-29 15:33:42
 * @LastEditTime: 2019-04-29 16:28:58
 */
#include "lidar_euclidean_cluster.h"

namespace LidarDetector {
/**
 * @description: 去除地面 
 */
void LidarClusterDetector::removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& only_floor_cloud,
    const double& max_height, const double& floor_max_angle)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr indexs(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // 设置分割对象是垂直平面
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    // 设置随机采样方式
    seg.setMethodType(pcl::SAC_RANSAC);
    // 设置最大迭代次数
    seg.setMaxIterations(100);
    // 设置垂直的轴
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    // 设置垂直角度的最大阈值
    seg.setEpsAngle(floor_max_angle);
    // 设置查询点到目标模型的最大距离
    seg.setDistanceThreshold(max_height);
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud);
    seg.segment(*indexs, *coefficients);

    if (indexs->indices.size() == 0) {
        printf("%s\n", "[lidar_euclidean_cluster_node]: could't seg floor");
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(indexs);
    extract.setNegative(true); // true removes the indices, false leaves only the indices
    extract.filter(*out_cloud);

    extract.setNegative(false); // true removes the indices, false leaves only the indices
    extract.filter(*only_floor_cloud);
}

/**
 * @description: 基于ray_groud_filtered对地面进行分割 
 */
void LidarClusterDetector::removeFloorRayFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_only_ground_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_no_ground_cloud,
    const double& sensor_height, const double& local_max_slope, const double& general_max_slope)
{
    pcl::PointIndices only_ground_indices;
    out_only_ground_cloud->points.clear();
    out_no_ground_cloud->points.clear();

    std::vector<PointCloudXYZRT> radial_divided_cloud;

    convertXYZ2XYZRT(in_cloud, radial_divided_cloud);

#pragma omp for
    for (size_t i = 0; i < radial_divided_cloud.size(); i++) {
        float prev_radius = 0.0;
        float prev_height = -sensor_height;
        bool prev_ground = false;
        bool current_ground = false;

        for (size_t j = 0; j < radial_divided_cloud[i].size(); j++) {
            float local_twoPoints_dis = radial_divided_cloud[i][j].radius - prev_radius;
            float local_height_threshold = tan(local_max_slope * M_PI / 180.) * local_twoPoints_dis;
            float general_height_threshold = tan(general_max_slope * M_PI / 180.) * radial_divided_cloud[i][j].radius;
            float current_height = radial_divided_cloud[i][j].point.z;

            if (radial_divided_cloud[i][j].radius > concentric_divide_distance && local_height_threshold < min_local_height_threshold) {
                local_height_threshold = min_local_height_threshold;
            }

            if (current_height <= (prev_height + local_height_threshold) && current_height >= (prev_height - local_height_threshold)) {
                if (!prev_ground) {
                    if (current_height <= (-sensor_height + general_height_threshold) && current_height >= (-sensor_height - general_height_threshold)) {
                        current_ground = true;
                    } else {
                        current_ground = false;
                    }
                } else {
                    current_ground = true;
                }
            } else {
                current_ground = false;
            }

            if (current_ground) {
                only_ground_indices.indices.push_back(radial_divided_cloud[i][j].original_index);
                prev_ground = true;
            } else {
                prev_ground = false;
            }
            prev_radius = radial_divided_cloud[i][j].radius;
            prev_height = radial_divided_cloud[i][j].point.z;
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(in_cloud);
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(only_ground_indices));

    extractor.setNegative(false); //true removes the indices, false leaves only the indices
    extractor.filter(*out_only_ground_cloud);

    extractor.setNegative(true); //true removes the indices, false leaves only the indices
    extractor.filter(*out_no_ground_cloud);
}

/**
 * @description: 截取点云，去除高度过高的点,去除距离激光雷达中心过近的点, 去除非车辆前面的点
 */
void LidarClusterDetector::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    const double& height, const double& near_dis, const double& far_dis,
    const double& left_right_dis)
{
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(in_cloud);
    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        double dis;
        // 计算 需要移除的点
        if (in_cloud->points[i].z > height) {
            indices.indices.push_back(i);
        } else if (in_cloud->points[i].x < 0 || in_cloud->points[i].y > left_right_dis || in_cloud->points[i].y < -left_right_dis) { // 激光雷达 x正方向朝前，y正方向朝左，z正方向朝上
            indices.indices.push_back(i);
        } else if ((dis = sqrt(pow(in_cloud->points[i].x, 2) + pow(in_cloud->points[i].y, 2))) < near_dis || dis > far_dis) {
            indices.indices.push_back(i);
        }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true); //true removes the indices, false leaves only the indices
    extractor.filter(*out_cloud);
}

/**
 * @description: 点云下采样
 */
void LidarClusterDetector::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    const double& leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(in_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*out_cloud);
}

/**
 * @description: 非结构化点云分割：根据大尺度范围的法向量和小尺度范围内的法向量的差异，去除了差异变化不明显的点(近似平面的点)
 * 参考链接：http://pointclouds.org/documentation/tutorials/don_segmentation.php
 */
void LidarClusterDetector::differenceOfNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    // 对于深度图这种结构化的数据，使用OrganizedNeighbor作为查找树
    // 对于激光雷达产生的非结构化数据，使用KdTree作为查找树
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    // 为查找树添加点云数据
    tree->setInputCloud(in_cloud);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_eatimation;
    // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;

    normal_eatimation.setInputCloud(in_cloud);
    normal_eatimation.setSearchMethod(tree);

    // setting viewpoint is very important, so that we can ensure that normals
    // estimated at different scales share a consistent orientation.
    normal_eatimation.setViewPoint(std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    pcl::PointCloud<pcl::PointNormal>::Ptr normal_small_scale(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_large_scale(new pcl::PointCloud<pcl::PointNormal>);
    // calculate normals with the small scale
    normal_eatimation.setRadiusSearch(small_scale);
    normal_eatimation.compute(*normal_small_scale);
    // calculate normals with the large scale
    normal_eatimation.setRadiusSearch(large_scale);
    normal_eatimation.compute(*normal_large_scale);

    // Create and initial the output cloud for DoN (Difference of Normals) results
    pcl::PointCloud<pcl::PointNormal>::Ptr diff_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud, *diff_normal_cloud);

    // Create DoN operator
    // The pcl::DifferenceOfNormalsEstimation class has 3 template parameters,
    // the first corresponds to the input point cloud type, in this case
    // pcl::PointXYZ, the second corresponds to the type of the normals
    // estimated for the point cloud, in this case pcl::PointNormal, and the
    // third corresponds to the vector field output type, in this case also
    // pcl::PointNormal.
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diff_normal_estimator;
    diff_normal_estimator.setInputCloud(in_cloud);
    diff_normal_estimator.setNormalScaleSmall(normal_small_scale);
    diff_normal_estimator.setNormalScaleLarge(normal_large_scale);
    diff_normal_estimator.initCompute();

    diff_normal_estimator.computeFeature(*diff_normal_cloud);

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>);
    // 设置条件：curvature必须满足大于(greater than) 角度阈值 angle_threshold
    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
        new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(diff_normal_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr diff_normal_filtered_cloud(new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    cond_removal.filter(*diff_normal_filtered_cloud);

    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diff_normal_filtered_cloud, *out_cloud);
}
}