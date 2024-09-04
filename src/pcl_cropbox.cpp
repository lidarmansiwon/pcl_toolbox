#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

class PCL_CropBox : public rclcpp::Node
{
public:
    PCL_CropBox()
        : Node("PCL_CropBox")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/agent3/points", rclcpp::SensorDataQoS(), std::bind(&PCL_CropBox::lidar_callback, this, std::placeholders::_1));

        pub_cropbox_ = create_publisher<sensor_msgs::msg::PointCloud2>("/crop_points",100);
    }

private:

    double crop_box_z_min_;
    double crop_box_z_max_;
    double crop_box_x_min_;
    double crop_box_x_max_;
    double crop_box_y_min_;
    double crop_box_y_max_;
    
    void declare_parameters()
    {   
        declare_parameter<double>("crop_box_y_min", -50.0);
        declare_parameter<double>("crop_box_y_max", 50.0);
        declare_parameter<double>("crop_box_z_min", 0.0);
        declare_parameter<double>("crop_box_z_max", 10.0);
        declare_parameter<double>("crop_box_x_min", 3.0);
        declare_parameter<double>("crop_box_x_max", 20.0);
    }

    void get_parameters()
    {   
        get_parameter("crop_box_y_min", crop_box_y_min_);
        get_parameter("crop_box_y_max", crop_box_y_max_);
        get_parameter("crop_box_z_min", crop_box_z_min_);
        get_parameter("crop_box_z_max", crop_box_z_max_);
        get_parameter("crop_box_x_min", crop_box_x_min_);
        get_parameter("crop_box_x_max", crop_box_x_max_); 
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr applyCropBoxFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        pcl::CropBox<pcl::PointXYZI> crop_box;
        crop_box.setInputCloud(input_cloud);
        crop_box.setMin(Eigen::Vector4f(crop_box_x_min_, crop_box_y_min_, crop_box_z_min_, 1.0));
        crop_box.setMax(Eigen::Vector4f(crop_box_x_max_, crop_box_y_max_, crop_box_z_max_, 1.0));

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        crop_box.filter(*output_cloud);

        return output_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    return cloud;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // CropBox filter
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped = applyCropBoxFilter(cloud);

        // Publish the Data
        sensor_msgs::msg::PointCloud2 cropped;
        pcl::toROSMsg(*cloud_cropped, cropped);
        pub_cropbox_->publish(cropped);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cropbox_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCL_CropBox>());
    rclcpp::shutdown();
    return 0;
}
