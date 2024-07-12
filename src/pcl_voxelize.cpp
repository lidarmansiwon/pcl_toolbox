#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

class PCL_Voxelize : public rclcpp::Node
{
public:
    PCL_Voxelize()
        : Node("PCL_Voxelize")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PCL_Voxelize::lidar_callback, this, std::placeholders::_1));

        pub_voxelize_ = create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_points",100);
    }

private:

    double voxel_resolution_;

    void declare_parameters()
    {   
        declare_parameter<double>("voxel_resolution", 0.5);
    }

    void get_parameters()
    {   
        get_parameter("voxel_resolution", voxel_resolution_); 
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    return cloud;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZI> downsample;
        downsample.setInputCloud(cloud);
        downsample.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_); // Set the voxel size
        downsample.filter(*cloud_downsampled);
        // Publish the Data
        sensor_msgs::msg::PointCloud2 downsampled;
        pcl::toROSMsg(*cloud_downsampled, downsampled);
        pub_voxelize_->publish(downsampled);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxelize_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCL_Voxelize>());
    rclcpp::shutdown();
    return 0;
}
