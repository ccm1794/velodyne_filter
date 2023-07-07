#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
//#include "velodyne_filter/msgVelodyne.h"
#include <geometry_msgs/msg/point.hpp>
#include <limits>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// etc
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>

using PointT = pcl::PointXYZI;

class VelodyneCluster : public rclcpp::Node
{
public:
  VelodynCluster()
  : Node("Velodyne_cluster")
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne Cluster Node has been started");

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points_filtered", 100);
    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_VoxelGrid", 100,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        LiDARCallback(msg);
      });
  }
  ~VelodyneCluster()
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne Cluster Node has been terminated");
  }

public:
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
};

void VelodyneCluster::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud_filtered);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree.setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndiced> cluster_indices;
  pcl::EuclideanClusterExtraction<pointT> ECE;
  ECE.setClusterTolerance(0.5); //m단위
  ECE.setMinClusterSize(1); // 몇 개부터 한 군집?
  ECE.setMaxClusterSize(2000); // 몇 개까지 한 군집?
  ECE.setSearchMethod(tree);
  ECE.setInputCloud(cloud_filtered);
  ECE.extract(cluster_indices);

  
}