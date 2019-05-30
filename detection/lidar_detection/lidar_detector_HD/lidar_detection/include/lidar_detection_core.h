#ifndef LIDAR_DETECTION_CORE_H_
#define LIDAR_DETECTION_CORE_H_
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/common.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <grid_map_cv/grid_map_cv.hpp>
// #include <grid_map_msgs/GridMap.h>
// #include <grid_map_ros/grid_map_ros.hpp>

// #include <vector_map/vector_map.h>

#include <tf/tf.h>

#include <yaml-cpp/yaml.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)

#include "gencolors.cpp"

#else

#include <autoware_msgs/DetectedObjectArray.h>
#include <opencv2/contrib/contrib.hpp>

#endif

#include "cluster_hd.h"

#ifdef GPU_CLUSTERING

#include "gpu_euclidean_clustering.h"

#endif

using namespace cv;

namespace LIDAR_DETECTION {
class lidar_detect {
private:
    ros::NodeHandle nh, pnh;
    ros::Publisher _pub_cluster_cloud;
    // ros::Publisher _pub_ground_cloud;
    ros::Publisher _centroid_pub;
    ros::Publisher _pub_clusters_message;
    ros::Publisher _pub_detected_objects;

    ros::Subscriber points_sub;

    std_msgs::Header _velodyne_header;
    std::string _output_frame;

    bool _downsample_cloud;
    bool _pose_estimation;
    double _leaf_size;
    int _cluster_size_min;
    int _cluster_size_max;

    bool _use_diffnormals;
    bool _use_vector_map;

    // double _clip_min_height; // 如果接收/ray_filter/velodyne_points_costmap, 则不需要再进行高度截取
    // double _clip_max_height;
    bool _keep_lanes;
    double _keep_lane_left_distance;
    double _keep_lane_right_distance;

    double _max_boundingbox_side;
    double _remove_points_upto;
    double _cluster_merge_threshold;
    double _clustering_distance;

    bool _use_gpu;

    std::vector<cv::Scalar> _colors;
    pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;
    visualization_msgs::Marker _visualization_marker;

    bool _use_multiple_thres;
    std::vector<double> _clustering_distances;
    std::vector<double> _clustering_ranges;

    tf::TransformListener* _transform_listener;
    tf::StampedTransform* _transform;

    void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2);

    void differenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);

    void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
        autoware_msgs::Centroids& in_out_centroids, autoware_msgs::CloudClusterArray& in_out_clusters);

    std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
        autoware_msgs::Centroids& in_out_centroids,
        double in_max_cluster_distance = 0.5);

    void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr>& in_clusters,
        std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
        double in_merge_threshold);

    void mergeClusters(const std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
        std::vector<size_t> in_merge_indices, const size_t& current_index,
        std::vector<bool>& in_out_merged_clusters);

    void checkAllForMerge(std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
        float in_merge_threshold);

    void publishColorCloud(const ros::Publisher* in_publisher,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);

    void publishCentroids(const ros::Publisher* in_publisher, const autoware_msgs::Centroids& in_centroids,
        const std::string& in_target_frame, const std_msgs::Header& in_header);

    void publishCloudClusters(const ros::Publisher* in_publisher, const autoware_msgs::CloudClusterArray& in_clusters,
        const std::string& in_target_frame, const std_msgs::Header& in_header);

    void publishDetectedObjects(const autoware_msgs::CloudClusterArray& in_clusters);

    std::vector<ClusterPtr> clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
        autoware_msgs::Centroids& in_out_centroids,
        double in_max_cluster_distance = 0.5);

public:
    lidar_detect(ros::NodeHandle n, ros::NodeHandle pn)
        : nh(n)
        , pnh(pn){};
    ~lidar_detect(){};

    void init();
};
}

#endif