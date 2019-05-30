#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <vector>
#include <dynamic_reconfigure/server.h>
#include <map_filter/FilterConfig.h>
#include <pcl/filters/voxel_grid.h>


namespace ASSIST_MAP_FILTER
{
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Subscriber sub_pc;
ros::Publisher pub_pc;

PointCloudT::Ptr base_map(new PointCloudT());
PointCloudT::Ptr map_filtered(new PointCloudT());
std::string in_pcd_file, out_pcd_file;
double leaf_size;

void load_map(std::string file){
  pcl::io::loadPCDFile(file, *base_map);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*base_map, msg);
  msg.header.frame_id = "map";
  pub_pc.publish(msg);
}

void points_cb(){
  map_filtered->clear();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_filter.setInputCloud(base_map);
  voxel_grid_filter.filter(*map_filtered);

  std::cout << "filtered map size = " << map_filtered->size() << std::endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*map_filtered,msg);
  msg.header.frame_id = "map";
  pub_pc.publish(msg);
}

void writeback_pc(){
  std::cout << "write back pc to " << out_pcd_file << std::endl;
  pcl::io::savePCDFile(out_pcd_file.c_str(), *map_filtered);
  std::cout << "Done" << std::endl;
}

void cfgcb(const map_filter::FilterConfig &config, uint32_t level){
    if(config.writeback){
      writeback_pc();
      return;
    }
    leaf_size = config.leaf_size;
    std::cout << "update leaf_size to: " << leaf_size << std::endl;
    points_cb();
}

void init(){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<double>("leaf_size", leaf_size, 1.0);
  pnh.param<std::string>("in_pcd_file", in_pcd_file, "config out_pcd_file");
  pnh.param<std::string>("out_pcd_file", out_pcd_file, "config out_pcd_file");
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/filter/map", 100);

  load_map(in_pcd_file);
}
}

using namespace ASSIST_MAP_FILTER;

int main(int argc, char** argv){
  ros::init(argc, argv, "map_filter_node");
  init();

  dynamic_reconfigure::Server<map_filter::FilterConfig> cfg_server;
  dynamic_reconfigure::Server<map_filter::FilterConfig>::CallbackType cfg_callback = boost::bind(&cfgcb, _1, _2);
  cfg_server.setCallback(cfg_callback);

  ros::spin();
  return 0;
}