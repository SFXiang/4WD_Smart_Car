#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void getAllFiles(const std::string path, std::vector<std::string> &files)
{
  DIR *dir;
  struct dirent *ptr;
  char base[1000];
  if ((dir = opendir(path.c_str())) == NULL)
  {
    perror("Open dir error...");
    std::cout << "Check: " << path << std::endl;
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (ptr->d_type == 8)
      files.push_back(ptr->d_name);
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "densy_map");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/densy_map", 1);
  sensor_msgs::PointCloud2 msg_map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZ>());
  ros::Duration duration;
  double param_duration;
  std::string param_pcd_path;
  pnh.param<double>("duration", param_duration, 1.0);
  pnh.param<std::string>("pcd_file_path", param_pcd_path, "null");
  std::cout << "[static_map] densy file path: " << param_pcd_path << std::endl;
  std::cout << "[static_map] duration: " << param_duration << std::endl;

  std::vector<std::string> files;
  getAllFiles(param_pcd_path, files);

  for (auto f : files)
  {
    // sensor_msgs::PointCloud2 temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());
    std::stringstream ss;
    ss << param_pcd_path << f;
    std::cout << "loading... " << ss.str() << std::endl;
    if (pcl::io::loadPCDFile(ss.str(), *temp) == -1)
    {
      ROS_ERROR("Failed to load pcd file: %s", ss.str());
      return (-1);
    }
    *pcl_map += *temp;
    std::cout << "loaded " << ss.str() << " with " << temp->size() << " points" << std::endl;
    ros::Duration(0.5).sleep();
  }

  pcl::toROSMsg(*pcl_map, msg_map);
  msg_map.header.frame_id = "map";

  duration.fromSec(param_duration);
  ros::Duration(1.0).sleep();

  while (ros::ok())
  {
    // msg_map.header.stamp = ros::Time::now();
    ROS_WARN_STREAM("publish densy map with " << msg_map.width);
    pub_map.publish(msg_map);
    duration.sleep();
    break; // 只发布一次
  }

  return 0;
}