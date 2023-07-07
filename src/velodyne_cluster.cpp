// 브랜치
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

#include <visualization_msgs/msg/marker_array.hpp>
#include <typeinfo>
#include <cstdio>

using namespace std;
using PointT = pcl::PointXYZI;

class VelodyneCluster : public rclcpp::Node
{
public:
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::PointCloud2 output;
  char mystr[10];
  
public:
  VelodyneCluster()
  : Node("velodyne_cluster")
  {
    RCLCPP_INFO(this->get_logger(), "Velodyne Cluster Node has been started");

    marker_Pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/velodyne_bbox", 100);
    LiDAR_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points_filtered", 100);
    LiDAR_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_ROI", 100,
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_Pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub_;
};

void VelodyneCluster::LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // pointcloud 데이터를 pcl::PointCloud로 변환
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud_filtered);

  // 유클리디언 클러스터 추출
  // 추출을 위한 KdTree 객체 생성
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ECE;
  ECE.setClusterTolerance(0.4); // 1m
  ECE.setMinClusterSize(10); // 몇 개부터 한 군집?
  ECE.setMaxClusterSize(100); // 몇 개까지 한 군집?
  ECE.setSearchMethod(tree);
  ECE.setInputCloud(cloud_filtered);
  ECE.extract(cluster_indices);

  // velodyne_filter::PointArray cluster_info;

  int j = 0;
  pcl::PointCloud<PointT> TotalCloud;
  std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;

  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
  {
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cluster->points.push_back(cloud_filtered->points[*pit]);
      PointT pt = cloud_filtered->points[*pit];
      PointT pt2;

      pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
      pt2.intensity = (float)(j+1);

      TotalCloud.push_back(pt2);

    }
    cluster->width = cluster->size();
    cluster->height +1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }
  cout << clusters.size() << endl;
  for(int i=0; i<clusters.size(); i++)
  {
    Eigen::Vector4f centroid;
    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;
    Eigen::Vector3f scale_;

    pcl::compute3DCentroid(*clusters[i], centroid);
    pcl::getMinMax3D(*clusters[i], min_p, max_p); //min_p와 max_p는 vector4f이므로 출력은 min_p[0] 이런 식으로 뽑을 수 있다.

    geometry_msgs::msg::Point center_point;
    center_point.x = centroid[0];
    center_point.y = centroid[1];
    center_point.z = centroid[2];

    geometry_msgs::msg::Point min_point;
    min_point.x = min_p.x(); //이렇게 해도 되네?
    min_point.y = min_p[1];
    min_point.z = min_p[2];

    geometry_msgs::msg::Point max_point;
    max_point.x = max_p[0];
    max_point.y = max_p[1];
    max_point.z = max_p[2];


    float width = max_p[0] - min_p[0];
    float height = max_p[1] - min_p[1];
    float depth = max_p[2] - min_p[2];

    scale_[0] = width; scale_[1] = height; scale_[2] = depth;

    Eigen::Quaternionf rotation(0.0, 0.0, 0.0, 1.0);
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = rotation.x();
    quaternion.y = rotation.y();
    quaternion.z = rotation.z();
    quaternion.w = rotation.w();

    pcl::PointXYZ boxCenter(centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ boxDimensions(width, height, depth);
    pcl::PointXYZ boxOrientation(rotation.x(), rotation.y(), rotation.z());

    
    this->marker.header.frame_id = "velodyne";
    this->marker.ns = "my_marker";
    this->marker.id = i;
    this->marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    this->marker.action = visualization_msgs::msg::Marker::ADD;
    this->marker.type = visualization_msgs::msg::Marker::CUBE;

    this->marker.scale.x = width;
    this->marker.scale.y = height;
    this->marker.scale.z = depth; //스케일은 벡터 형태로?
    //this->marker.scale = scale_; // 두 방법 모두 가능하다

    this->marker.color.r = 0.5;
    this->marker.color.g = 0.5;
    this->marker.color.b = 0.5;
    this->marker.color.a = 0.5; // 0~1사이 값
    //this->marker.color = std_msgs::msg::ColorRGBA(0.5, 0.5, 0.5, 0.8); //위의 네 줄을 이렇게 해도 된다?

    // this->marker.pose.orientation = quaternion.x;
    // this->marker.pose.orientation = quaternion.y;
    // this->marker.pose.orientation = quaternion.z;
    // this->marker.pose.orientation = quaternion.w;
    this->marker.pose.orientation = quaternion;

    // this->marker.pose.position = center_point.x;
    // this->marker.pose.position = center_point.y;
    // this->marker.pose.position = center_point.z;
    this->marker.pose.position = center_point;

    // sprintf(this->mystr, "%g", i);
    // this->marker.text = this->mystr;

    this->markerArray.markers.push_back(this->marker);
    
  }
  
  // this->markerArray.markers.push_back(this->marker);
  this->marker_Pub_->publish(this->markerArray);
  this->markerArray.markers.clear();

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);

  pcl_conversions::fromPCL(cloud_p, this->output);

  output.header.frame_id = "velodyne";

  this->LiDAR_pub_->publish(this->output);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelodyneCluster>());
  rclcpp::shutdown();
  return 0;
}

// 이 코드는 kdtree에 값을 넣을 때 데이터에 inf 나 nan 값이 있으면 오류가 뜨는 듯 하다.
// 이것은 plz코드에 있는 385번째 줄부터 시작하는 범위를 제한하는 것으로 문제를 해결할 수 있다.