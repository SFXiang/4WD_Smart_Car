#ifndef LIDAR_DETECTION_CORE_CPP_
#define LIDAR_DETECTION_CORE_CPP_
#include "lidar_detection_core.h"
// #include "gpu_euclidean_clustering.h"

namespace LIDAR_DETECTION {
void lidar_detect::init()
{
#if (CV_MAJOR_VERSION == 3)
    generateColors(_colors, 255);
#else
    cv::generateColors(_colors, 255);
#endif

    _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
    // _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
    _centroid_pub = nh.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);
    _pub_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
    _pub_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    std::string points_topic, gridmap_topic;
    // _using_sensor_cloud = false;  // we use /ray_filter/velodyne_points_costmap

    tf::TransformListener listener;
    tf::StampedTransform transform;
    _transform_listener = &listener;
    _transform = &transform;

    pnh.param<std::string>("points_topic", points_topic, "confirm in_points_topic");
    pnh.param<bool>("use_diffnormals", _use_diffnormals, false);
    pnh.param<bool>("downsample_cloud", _downsample_cloud, false);
    pnh.param<double>("leaf_size", _leaf_size, 0.1);
    pnh.param<int>("cluster_size_min", _cluster_size_min, 20);
    pnh.param<int>("cluster_size_max", _cluster_size_max, 100000);
    pnh.param<bool>("pose_estimation", _pose_estimation, false);
    // private_nh.param("clip_min_height", _clip_min_height, -1.3);
    // private_nh.param("clip_max_height", _clip_max_height, 0.5);
    pnh.param<bool>("keep_lanes", _keep_lanes, false);
    pnh.param<double>("keep_lane_left_distance", _keep_lane_left_distance, 5.0);
    pnh.param<double>("keep_lane_right_distance", _keep_lane_right_distance, 5.0);
    pnh.param<double>("max_boundingbox_side", _max_boundingbox_side, 10.0);
    pnh.param<double>("cluster_merge_threshold", _cluster_merge_threshold, 1.5);
    pnh.param<std::string>("output_frame", _output_frame, "velodyne");

    pnh.param<double>("remove_points_upto", _remove_points_upto, 0.0);
    pnh.param<double>("clustering_distance", _clustering_distance, 0.75);
    pnh.param<bool>("use_gpu", _use_gpu, false);
    pnh.param<bool>("use_multiple_thres", _use_multiple_thres, false);

    std::string str_distances; // 用于多阈值cluster检测
    std::string str_ranges;

    pnh.param<std::string>("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
    pnh.param<std::string>("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));

    if (_use_multiple_thres) {
        YAML::Node distances = YAML::Load(str_distances);
        YAML::Node ranges = YAML::Load(str_ranges);
        size_t distances_size = distances.size();
        size_t ranges_size = ranges.size();
        if (distances_size == 0 || ranges_size == 0) {
            ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    The size of clustering distance and clustering_ranges should not be 0");
            ros::shutdown();
        }
        if ((distances_size - ranges_size) != 1) {
            ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    Expecting that (distances_size - ranges_size) == 1 ");
            ros::shutdown();
        }
        for (size_t i_distance = 0; i_distance < distances_size; i_distance++) {
            _clustering_distances.push_back(distances[i_distance].as<double>());
        }
        for (size_t i_range = 0; i_range < ranges_size; i_range++) {
            _clustering_ranges.push_back(ranges[i_range].as<double>());
        }
    }

    points_sub = nh.subscribe(points_topic, 1, &lidar_detect::velodyne_callback, this);

    ros::spin();
}

void lidar_detect::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
    //_start = std::chrono::system_clock::now();
    static bool _using_sensor_cloud = false;

    if (!_using_sensor_cloud) {
        _using_sensor_cloud = true;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        autoware_msgs::Centroids centroids;
        autoware_msgs::CloudClusterArray cloud_clusters;

        // pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
        pcl::fromROSMsg(*in_sensor_cloud, *nofloor_cloud_ptr);

        _velodyne_header = in_sensor_cloud->header;

        // if (_remove_points_upto > 0.0) {
        //     removePointsUpTo(current_sensor_cloud_ptr, removed_points_cloud_ptr, _remove_points_upto);
        // } else
        //     removed_points_cloud_ptr = current_sensor_cloud_ptr;

        if (_downsample_cloud)
            downsampleCloud(nofloor_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
        else
            downsampled_cloud_ptr = nofloor_cloud_ptr;

        // clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height);

        // if (_keep_lanes) // TODO:: 只提取保留可行驶车道内的点
        //     keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance);
        // else
        //     inlanes_cloud_ptr = clipped_cloud_ptr;

        // if (_remove_ground) {
        //     removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr); // 平面拟合法--实际使用ray_ground_filter更好
        //     publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
        // } else
        //     nofloor_cloud_ptr = inlanes_cloud_ptr;

        // publishCloud(&_pub_points_lanes_cloud, nofloor_cloud_ptr);

        if (_use_diffnormals) // ????
            differenceNormalsSegmentation(downsampled_cloud_ptr, diffnormals_cloud_ptr);
        else
            diffnormals_cloud_ptr = nofloor_cloud_ptr;

        // 主函数
        segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, centroids,
            cloud_clusters);

        publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr);

        centroids.header = _velodyne_header;

        publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);

        cloud_clusters.header = _velodyne_header;

        publishCloudClusters(&_pub_clusters_message, cloud_clusters, _output_frame, _velodyne_header);

        _using_sensor_cloud = false;
    }
}

void lidar_detect::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}

void lidar_detect::differenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    float small_scale = 0.5;
    float large_scale = 2.0;
    float angle_threshold = 0.5;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (in_cloud_ptr->isOrganized()) {
        tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    } else {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(in_cloud_ptr);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    normal_estimation.setInputCloud(in_cloud_ptr);
    normal_estimation.setSearchMethod(tree);

    normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

    normal_estimation.setRadiusSearch(small_scale);
    normal_estimation.compute(*normals_small_scale);

    normal_estimation.setRadiusSearch(large_scale);
    normal_estimation.compute(*normals_large_scale);

    pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud_ptr, *diffnormals_cloud);

    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
    diffnormals_estimator.setInputCloud(in_cloud_ptr);
    diffnormals_estimator.setNormalScaleLarge(normals_large_scale);
    diffnormals_estimator.setNormalScaleSmall(normals_small_scale);

    diffnormals_estimator.initCompute();

    diffnormals_estimator.computeFeature(*diffnormals_cloud);

    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
        new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(diffnormals_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    cond_removal.filter(*diffnormals_cloud_filtered);

    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diffnormals_cloud, *out_cloud_ptr);
}

void lidar_detect::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids& in_out_centroids, autoware_msgs::CloudClusterArray& in_out_clusters)
{
    // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
    // entire pc)
    // in this way, the points farther in the pc will also be clustered

    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6

    std::vector<ClusterPtr> all_clusters;

    if (!_use_multiple_thres) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
            pcl::PointXYZ current_point;
            current_point.x = in_cloud_ptr->points[i].x;
            current_point.y = in_cloud_ptr->points[i].y;
            current_point.z = in_cloud_ptr->points[i].z;

            cloud_ptr->points.push_back(current_point);
        }
#ifdef GPU_CLUSTERING
        if (_use_gpu) {
            all_clusters = clusterAndColorGpu(cloud_ptr, out_cloud_ptr, in_out_centroids,
                _clustering_distance);
        } else {
            all_clusters = clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);
        }
#else
        all_clusters = clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);
#endif
    } else {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_segments_array[i] = tmp_cloud;
        }

        for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
            pcl::PointXYZ current_point;
            current_point.x = in_cloud_ptr->points[i].x;
            current_point.y = in_cloud_ptr->points[i].y;
            current_point.z = in_cloud_ptr->points[i].z;

            float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

            if (origin_distance < _clustering_ranges[0]) {
                cloud_segments_array[0]->points.push_back(current_point);
            } else if (origin_distance < _clustering_ranges[1]) {
                cloud_segments_array[1]->points.push_back(current_point);
            } else if (origin_distance < _clustering_ranges[2]) {
                cloud_segments_array[2]->points.push_back(current_point);
            } else if (origin_distance < _clustering_ranges[3]) {
                cloud_segments_array[3]->points.push_back(current_point);
            } else {
                cloud_segments_array[4]->points.push_back(current_point);
            }
        }

        std::vector<ClusterPtr> local_clusters;
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++) {
#ifdef GPU_CLUSTERING
            if (_use_gpu) {
                local_clusters = clusterAndColorGpu(cloud_segments_array[i], out_cloud_ptr,
                    in_out_centroids, _clustering_distances[i]);
            } else {
                local_clusters = clusterAndColor(cloud_segments_array[i], out_cloud_ptr,
                    in_out_centroids, _clustering_distances[i]);
            }
#else
            local_clusters = clusterAndColor(
                cloud_segments_array[i], out_cloud_ptr, in_out_centroids, _clustering_distances[i]);
#endif
            all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
        }
    }

    // Clusters can be merged or checked in here
    //....
    // check for mergable clusters
    std::vector<ClusterPtr> mid_clusters;
    std::vector<ClusterPtr> final_clusters;

    if (all_clusters.size() > 0)
        checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
    else
        mid_clusters = all_clusters;

    if (mid_clusters.size() > 0)
        checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
    else
        final_clusters = mid_clusters;

    //     tf::StampedTransform vectormap_transform;
    //     if (_use_vector_map) {
    //         if (_wayarea_gridmap.exists(_gridmap_layer)) {
    //             // check if centroids are inside the drivable area
    //             cv::Mat grid_image;
    //             grid_map::GridMapCvConverter::toImage<unsigned char, 1>(_wayarea_gridmap, _gridmap_layer, CV_8UC1,
    //                 _grid_min_value, _grid_max_value, grid_image);

    // #pragma omp for
    //             for (unsigned int i = 0; i < final_clusters.size(); i++) {
    //                 pcl::PointXYZ pcl_centroid = final_clusters[i]->GetCentroid();

    //                 geometry_msgs::Point original_centroid_point, final_centroid_point;
    //                 original_centroid_point.x = pcl_centroid.x;
    //                 original_centroid_point.y = pcl_centroid.y;
    //                 original_centroid_point.z = pcl_centroid.z;

    //                 if (_wayarea_gridmap.getFrameId() != _velodyne_header.frame_id) {
    //                     tf::StampedTransform grid_sensor_tf = findTransform(_wayarea_gridmap.getFrameId(),
    //                         _velodyne_header.frame_id);
    //                     final_centroid_point = transformPoint(original_centroid_point, grid_sensor_tf);
    //                 } else {
    //                     final_centroid_point = original_centroid_point;
    //                 }

    //                 bool point_in_grid = checkPointInGrid(_wayarea_gridmap, grid_image, final_centroid_point);
    //                 final_clusters[i]->SetValidity(point_in_grid);
    //             }
    //             // timer.stop();
    //             // std::cout << "vectormap filtering took " << timer.getTimeMilli() << " ms to check " << final_clusters.size() <<
    //             // std::endl;
    //         } else {
    //             ROS_INFO("%s layer not contained in the OccupancyGrid", _gridmap_layer.c_str());
    //         }
    //     }
    // Get final PointCloud to be published
    for (unsigned int i = 0; i < final_clusters.size(); i++) {
        *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());
        pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
        geometry_msgs::Point centroid;
        centroid.x = center_point.x;
        centroid.y = center_point.y;
        centroid.z = center_point.z;

        if (final_clusters[i]->IsValid()) {

            in_out_centroids.points.push_back(centroid);

            autoware_msgs::CloudCluster cloud_cluster;
            final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
            in_out_clusters.clusters.push_back(cloud_cluster);
        }
    }
}

std::vector<ClusterPtr> lidar_detect::clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids& in_out_centroids,
    double in_max_cluster_distance)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
    // make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++) {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> cluster_indices;

    // perform clustering on 2d cloud
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(in_max_cluster_distance); //
    ec.setMinClusterSize(_cluster_size_min);
    ec.setMaxClusterSize(_cluster_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);
    // use indices on 3d cloud

    /////////////////////////////////
    //---	3. Color clustered points
    /////////////////////////////////
    unsigned int k = 0;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<ClusterPtr> clusters;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
    // cluster
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        ClusterPtr cluster(new Cluster());
        cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0],
            (int)_colors[k].val[1],
            (int)_colors[k].val[2], "", _pose_estimation);
        clusters.push_back(cluster);

        k++;
    }
    // std::cout << "Clusters: " << k << std::endl;
    return clusters;
}

void lidar_detect::checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr>& in_clusters,
    std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
    double in_merge_threshold)
{
    // std::cout << "checkClusterMerge" << std::endl;
    pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
    for (size_t i = 0; i < in_clusters.size(); i++) {
        if (i != in_cluster_id && !in_out_visited_clusters[i]) {
            pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
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

void lidar_detect::mergeClusters(const std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
    std::vector<size_t> in_merge_indices, const size_t& current_index,
    std::vector<bool>& in_out_merged_clusters)
{
    // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
    pcl::PointCloud<pcl::PointXYZ> mono_cloud;
    ClusterPtr merged_cluster(new Cluster());
    for (size_t i = 0; i < in_merge_indices.size(); i++) {
        sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    std::vector<int> indices(sum_cloud.points.size(), 0);
    for (size_t i = 0; i < sum_cloud.points.size(); i++) {
        indices[i] = i;
    }

    if (sum_cloud.points.size() > 0) {
        pcl::copyPointCloud(sum_cloud, mono_cloud);
        merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
            (int)_colors[current_index].val[0], (int)_colors[current_index].val[1],
            (int)_colors[current_index].val[2], "", _pose_estimation);
        out_clusters.push_back(merged_cluster);
    }
}

void lidar_detect::checkAllForMerge(std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
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

void lidar_detect::publishColorCloud(const ros::Publisher* in_publisher,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);
}

void lidar_detect::publishCentroids(const ros::Publisher* in_publisher, const autoware_msgs::Centroids& in_centroids,
    const std::string& in_target_frame, const std_msgs::Header& in_header)
{
    if (in_target_frame != in_header.frame_id) {
        autoware_msgs::Centroids centroids_transformed;
        centroids_transformed.header = in_header;
        centroids_transformed.header.frame_id = in_target_frame;
        for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++) {
            geometry_msgs::PointStamped centroid_in, centroid_out;
            centroid_in.header = in_header;
            centroid_in.point = *i;
            try {
                _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                    centroid_out);

                centroids_transformed.points.push_back(centroid_out.point);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("publishCentroids: %s", ex.what());
            }
        }
        in_publisher->publish(centroids_transformed);
    } else {
        in_publisher->publish(in_centroids);
    }
}

void lidar_detect::publishCloudClusters(const ros::Publisher* in_publisher, const autoware_msgs::CloudClusterArray& in_clusters,
    const std::string& in_target_frame, const std_msgs::Header& in_header)
{
    if (in_target_frame != in_header.frame_id) {
        autoware_msgs::CloudClusterArray clusters_transformed;
        clusters_transformed.header = in_header;
        clusters_transformed.header.frame_id = in_target_frame;
        for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++) {
            autoware_msgs::CloudCluster cluster_transformed;
            cluster_transformed.header = in_header;
            try {
                _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(),
                    *_transform);
                pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                    cluster_transformed.min_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                    cluster_transformed.max_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                    cluster_transformed.avg_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                    cluster_transformed.centroid_point);

                cluster_transformed.dimensions = i->dimensions;
                cluster_transformed.eigen_values = i->eigen_values;
                cluster_transformed.eigen_vectors = i->eigen_vectors;

                clusters_transformed.clusters.push_back(cluster_transformed);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("publishCloudClusters: %s", ex.what());
            }
        }
        in_publisher->publish(clusters_transformed);
        publishDetectedObjects(clusters_transformed);
    } else {
        in_publisher->publish(in_clusters);
        publishDetectedObjects(in_clusters);
    }
}

void lidar_detect::publishDetectedObjects(const autoware_msgs::CloudClusterArray& in_clusters)
{
    autoware_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_clusters.header;

    for (size_t i = 0; i < in_clusters.clusters.size(); i++) {
        autoware_msgs::DetectedObject detected_object;
        detected_object.header = in_clusters.header;
        detected_object.label = "unknown";
        detected_object.score = 1.;
        detected_object.space_frame = in_clusters.header.frame_id;
        detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
        detected_object.dimensions = in_clusters.clusters[i].dimensions;
        detected_object.pointcloud = in_clusters.clusters[i].cloud;
        detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
        detected_object.valid = true;

        detected_objects.objects.push_back(detected_object);
    }
    _pub_detected_objects.publish(detected_objects);
}

#ifdef GPU_CLUSTERING

std::vector<ClusterPtr> lidar_detect::clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids& in_out_centroids,
    double in_max_cluster_distance)
{
    std::vector<ClusterPtr> clusters;

    // Convert input point cloud to vectors of x, y, and z

    int size = in_cloud_ptr->points.size();

    if (size == 0)
        return clusters;

    float *tmp_x, *tmp_y, *tmp_z;

    tmp_x = (float*)malloc(sizeof(float) * size);
    tmp_y = (float*)malloc(sizeof(float) * size);
    tmp_z = (float*)malloc(sizeof(float) * size);

    for (int i = 0; i < size; i++) {
        pcl::PointXYZ tmp_point = in_cloud_ptr->at(i);

        tmp_x[i] = tmp_point.x;
        tmp_y[i] = tmp_point.y;
        tmp_z[i] = tmp_point.z;
    }

    GpuEuclideanCluster gecl_cluster;

    gecl_cluster.setInputPoints(tmp_x, tmp_y, tmp_z, size);
    gecl_cluster.setThreshold(in_max_cluster_distance);
    gecl_cluster.setMinClusterPts(_cluster_size_min);
    gecl_cluster.setMaxClusterPts(_cluster_size_max);
    gecl_cluster.extractClusters();
    std::vector<GpuEuclideanCluster::GClusterIndex> cluster_indices = gecl_cluster.getOutput();

    unsigned int k = 0;

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        ClusterPtr cluster(new Cluster());
        cluster->SetCloud(in_cloud_ptr, it->points_in_cluster, _velodyne_header, k, (int)_colors[k].val[0],
            (int)_colors[k].val[1], (int)_colors[k].val[2], "", _pose_estimation);
        clusters.push_back(cluster);

        k++;
    }

    free(tmp_x);
    free(tmp_y);
    free(tmp_z);

    return clusters;
}

#endif
}

#endif