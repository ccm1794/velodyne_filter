#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

using namespace std;
using PointT = pcl::PointXYZI;



class VelodyneROI : public rclcpp::Node
{
public:
  sensor_msgs::msg::PointCloud2 output;
  int flag = 0;
public:
  VelodyneROI()
  : Node("velodyne_ROI")
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne ROI Node has been started");

    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_ROI", 100);
    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_VoxelGrid", 100,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        LiDARCallback(msg);
      });
  }

  ~VelodyneROI()
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne ROI Node has been terminated");
  }

public:
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
};

void VelodyneROI::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud_filtered);

  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud(cloud_filtered);
  cropFilter.setMin(Eigen::Vector4f(0.0, -3.0, -0.5, 1.0));
  cropFilter.setMax(Eigen::Vector4f(4.0, 3.0, 3.0, 1.0));
  cropFilter.filter(*cloud_filtered);

  if(cloud_filtered->points.size() == 0)
  {
    this->flag = 0;
    cout << "조졌네 이거" << endl;
  }
  else
  {
    pcl::toROSMsg(*cloud_filtered, this->output);
    this->flag = 1;
    this->output.header.frame_id = "velodyne";
    this->output.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    //system("clear");
    this->LiDAR_pub_->publish(this->output);
  } 
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelodyneROI>());
  rclcpp::shutdown();
  return 0;
}