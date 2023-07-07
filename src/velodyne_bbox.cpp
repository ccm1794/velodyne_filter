#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZI;

class VelodyneBBox : public rclcpp::Node
{
public:
  VelodyneBBox()
  : Node("Velodyne_BBox")
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne BBox Node has been started");

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_Boxed", 100);
    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_filtered", 100,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        LiDARCallback(msg);
      });
  }

  ~VelodyneBBox()
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne BBox Node has been terminated");
  }

public:
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
};

void VelodyneBBox::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  // pcl::fromROSMsg(*msg, *cloud_filtered);

  // pcl::PCLPointCloud2 cloud_p;
  // pcl::toPCLPointCloud2(*cloud_filtered, cloud_p);

  // sensor_msgs::msg::PointCloud2 output;
  // pcl_conversions::fromPCL(cloud_p, output);

  // output.header.frame_id = "velodyne";

  // this->LiDAR_pub_->publish(output);

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud_filtered);

  
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelodyneBBox>());
  rclcpp::shutdown();
  return 0;
}