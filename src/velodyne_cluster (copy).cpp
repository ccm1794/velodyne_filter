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

using namespace std;
using PointT = pcl::PointXYZI;

class VelodyneCluster : public rclcpp::Node
{
public:
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker marker;
  sensor_msgs::msg::PointCloud2 output;
  
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
  ECE.setClusterTolerance(0.5); // 1m
  ECE.setMinClusterSize(1); // 몇 개부터 한 군집?
  ECE.setMaxClusterSize(2000); // 몇 개까지 한 군집?
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
  
  for(int i=0; i<clusters.size(); i++)
  {
    Eigen::Vector4f centroid;
    Eigen::Vector4f min_p;
    Eigen::Vector4f max_p;

    pcl::compute3DCentroid(*clusters[i], centroid);
    pcl::getMinMax3D(*clusters[i], min_p, max_p); //min_p와 max_p는 vector4f이므로 출력은 min_p[0] 이런 식으로 뽑을 수 있다.

    cout << min_p[0] << endl;
    cout << "====" << endl;

    //Define the eight vertices of the bounding box
    std::vector<pcl::PointXYZ> vertices;
    vertices.push_back(pcl::PointXYZ(min_p[0], min_p[1], min_p[2]));
    vertices.push_back(pcl::PointXYZ(max_p[0], min_p[1], min_p[2]));
    vertices.push_back(pcl::PointXYZ(max_p[0], max_p[1], min_p[2]));
    vertices.push_back(pcl::PointXYZ(min_p[0], max_p[1], min_p[2]));
    vertices.push_back(pcl::PointXYZ(min_p[0], max_p[1], max_p[2]));
    vertices.push_back(pcl::PointXYZ(min_p[0], min_p[1], max_p[2]));
    vertices.push_back(pcl::PointXYZ(max_p[0], min_p[1], max_p[2]));
    vertices.push_back(pcl::PointXYZ(max_p[0], max_p[1], max_p[2]));

    for(int p = 0; p < 4; ++p)
    {
      int j = (p + 1) % 4;
      int k = p + 4;
      int l = j + 4;

      geometry_msgs::msg::Point startPoint, endPoint;
      startPoint.x = vertices[p].x;
      startPoint.y = vertices[p].y;
      startPoint.z = vertices[p].z;

      endPoint.x = vertices[j].x;
      endPoint.y = vertices[j].y;
      endPoint.z = vertices[j].z;

      this->marker.points.push_back(startPoint);
      this->marker.points.push_back(endPoint);

      startPoint.x = vertices[k].x;
      startPoint.y = vertices[k].y;
      startPoint.z = vertices[k].z;

      endPoint.x = vertices[l].x;
      endPoint.y = vertices[l].y;
      endPoint.z = vertices[l].z;

      this->marker.points.push_back(startPoint);
      this->marker.points.push_back(endPoint);

      startPoint.x = vertices[p].x;
      startPoint.y = vertices[p].y;
      startPoint.z = vertices[p].z;

      endPoint.x = vertices[k].x;
      endPoint.y = vertices[k].y;
      endPoint.z = vertices[k].z;

      this->marker.points.push_back(startPoint);
      this->marker.points.push_back(endPoint);
    }

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

    Eigen::Quaternionf rotation(1.0, 0.0, 0.0, 0.0);

    pcl::PointXYZ boxCenter(centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ boxDimensions(width, height, depth);
    pcl::PointXYZ boxOrientation(rotation.x(), rotation.y(), rotation.z());
  }
  this->marker.header.frame_id = "velodyne";
  this->marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  this->marker.action = visualization_msgs::msg::Marker::ADD;
  this->marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  this->marker.scale.x = 0.02; // Set the line width
  
  this->markerArray.markers.push_back(this->marker);
  this->marker_Pub_->publish(this->markerArray);

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