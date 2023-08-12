#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class MyPointCloudProcessor : public rclcpp::Node
{
public:
    MyPointCloudProcessor() : Node("ransac")
    {
        pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_ransac", 1);
        pub_input_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 1);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 1,
             std::bind(&MyPointCloudProcessor::cloud_cb, this, std::placeholders::_1));

        // RANSAC parameters
        distance_threshold_ = 0.25;

        RCLCPP_INFO(this->get_logger(), "PointCloud processor node has started.");
        //std::cout << "0" << std::endl;

    }

private:
    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        //std::cout << "1" << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*input, *cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold_);
        
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Remove ground points
        removeGround(cloud, coefficients, distance_threshold_);

        // Publish the point cloud after ground removal
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = input->header;
        pub_output_->publish(output);

        //std::cout << "2" << std::endl;

        // Publish the original input point cloud with a different color for visualization
        // 이 아래는 없어도 됨
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *cloud_rgb);
        for (std::size_t i = 0; i < cloud_rgb->size(); ++i)
        {
            cloud_rgb->points[i].r = 255; // Set the color to red
            cloud_rgb->points[i].g = 0;
            cloud_rgb->points[i].b = 0;
        }
        sensor_msgs::msg::PointCloud2 input_rgb;
        pcl::toROSMsg(*cloud_rgb, input_rgb);
        input_rgb.header = input->header;
        pub_input_->publish(input_rgb);
    }

    void removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::ModelCoefficients::Ptr& coefficients, double distance_threshold)
    {
        //std::cout << "3" << std::endl;

        // Remove points close to the ground plane
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (std::size_t i = 0; i < cloud->size(); ++i)
        {
            double distance = std::abs(coefficients->values[0] * cloud->points[i].x +
                                       coefficients->values[1] * cloud->points[i].y +
                                       coefficients->values[2] * cloud->points[i].z +
                                       coefficients->values[3]) /
                              std::sqrt(coefficients->values[0] * coefficients->values[0] +
                                        coefficients->values[1] * coefficients->values[1] +
                                        coefficients->values[2] * coefficients->values[2]);
            if (distance < distance_threshold)
                inliers->indices.push_back(i);
        }

        //std::cout << "4" << std::endl;

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_input_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    double distance_threshold_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}