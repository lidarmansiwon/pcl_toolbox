#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

class PCL_SOR_Filter : public rclcpp::Node
{
public:
    PCL_SOR_Filter()
        : Node("PCL_SOR_Filter")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PCL_SOR_Filter::lidar_callback, this, std::placeholders::_1));

        pub_sor_filter = create_publisher<sensor_msgs::msg::PointCloud2>("/sor_filter_points",100);
    }

private:

    double setMean_;
    double setStddevMulThresh_;
    
    void declare_parameters()
    {   
        declare_parameter<double>("setMean", 10.0);
        declare_parameter<double>("setStddevMulThresh", 1.0);
    }

    void get_parameters()
    {   
        get_parameter("setMean", setMean_);
        get_parameter("setStddevMulThresh", setStddevMulThresh_);    
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    return cloud;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // pcl::fromROSMsg(*input, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // StatisticalOutlierRemoval filtering
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
        sor_filter.setInputCloud(cloud);
        sor_filter.setMeanK(setMean_);
        sor_filter.setStddevMulThresh(setStddevMulThresh_);
        sor_filter.filter(*cloud_filtered);

        // Publish the Data
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        pub_sor_filter->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sor_filter;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCL_SOR_Filter>());
    rclcpp::shutdown();
    return 0;
}
