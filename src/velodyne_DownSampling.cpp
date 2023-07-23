// header for ROS

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

// header for using pcl

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using PointT = pcl::PointXYZI;

class VelodyneVoxelGrid : public rclcpp::Node
{
public:
  sensor_msgs::msg::PointCloud2 output;
  int flag = 0;
public:
  VelodyneVoxelGrid()
  : Node("velodyne_VoxelGrid")
  {
    RCLCPP_INFO(this->get_logger(), "velodyne downSampling Node has been started");

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_VoxelGrid", 100);
    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 100,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        LiDARCallback(msg);
      });
  }
  ~VelodyneVoxelGrid()
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne VoxelGrid Node has been terminated");
  }
public:
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
};

void VelodyneVoxelGrid::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud_filtered);

  pcl::VoxelGrid<PointT> VG;
  VG.setInputCloud(cloud_filtered);
  // VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.setLeafSize(0.05f, 0.05f, 0.05f);
  VG.filter(*cloud_filtered);

  pcl::toROSMsg(*cloud_filtered, this->output);
  this->output.header.frame_id = "velodyne";
  this->output.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  //system("clear");
  this->LiDAR_pub_->publish(this->output);
}




int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelodyneVoxelGrid>());
    rclcpp::shutdown();
    return 0;
}
