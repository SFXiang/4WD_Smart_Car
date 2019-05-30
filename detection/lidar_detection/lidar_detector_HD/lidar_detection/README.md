# Lidar Cluster Detector
-- by LitoNeo

A ROS_Node which cluster the points cloud.

---

SmartCar package for ground filter
* From a sourced terminal:
    * For RC_car:
     --haven't done it, please verify the params in `Yunlecar_lidar_detection.launch` to create it
    * For Yunle_car:
    `roslaunch lidar_detection Yunlecar_lidar_detection.launch`
    * For test:
    `roslaunch lidar_detection test_Yunlecar_lidar_detection.launch` 
    > and this will launch `visualize_detected_objects` to visualize result at RVIZ

### Requirements
Sensor:
> velodyne_driver

Node:
> `ray_ground_filter`

### Subscribed topics
|Topic|Type|Objective|
|---|---|---|
|/velodyne_points_costmap|sensor_msgs::PointCloud2|original point-cloud from VLP16|

### Published topics
|Topic|Type|Objective|
|---|---|---|
|/points_cluster|sensor_msgs::PointCloud2|all the cluster points with different color|
|/cluster_centroids|autoware_msgs::Centroids|all the Centroids of clusters|
|/detection/lidar_detector/cloud_clusters|autoware_msgs::CloudClusterArray| |
|/detection/lidar_detector/objects|autoware_msgs::DetectedObjectArray|all the clusters in format of DetectedObject |

### Parameters
- [ ] TODO::edit it
```c++ 
    <param name="points_topic" value="/ray_filter/velodyne_points_costmap"/>
    <param name="use_diffnormals" value="false"/>
    <param name="downsample_cloud" value="false"/>
    <param name="leaf_size" value="0.1"/>
    <param name="cluster_size_min" value="20"/>
    <param name="cluster_size_max" value="100000" />
    <param name="pose_estimation" value="false" />
    <!-- <param name="clip_min_height" value="-2.0"/>
    <param name="clip_max_height" value="0.5"/> -->
    <param name="keep_lanes" value="false" />
    <param name="keep_lane_left_distance" value="5.0" />
    <param name="keep_lane_right_distance" value="5.0" />
    <param name="max_boundingbox_side" value="10.0"/>
    <param name="cluster_merge_threshold" value="1.5" />
    <param name="output_frame" value="velodyne" />
    <param name="remove_points_upto" value="0.0" />
    <param name="miclustering_distancenY" value="0.75" />
    <param name="use_gpu" value="false" />
    <param name="use_multiple_thres" value="true" />
    <param name="clustering_distances" value="[0.5,1.1,1.6,2.1,2.6]" />
    <param name="clustering_ranges" value="[15,30,45,60]"/>
```


### Process
 - [ ] To Do Later

### Probloms

### TODO list
- [ ] extract `wayerea` to conduct clustering only in road area
- [ ] use jsk-boundingbox to conduct visualization

### Reference
autoware -> lidar detection

### Video
<iframe height=450 width=800 src="https://youtu.be/xPse8F-uGzE" frameborder=0 allowfullscreen></iframe>

If failed to load the video, clip here: [YouTube](https://youtu.be/xPse8F-uGzE)