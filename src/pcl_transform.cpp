#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


class PCL_Transform : public rclcpp::Node
{
public:
    PCL_Transform()
        : Node("PCL_Transform")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&PCL_Transform::lidar_callback, this, std::placeholders::_1));

        pub_transformed_ = create_publisher<sensor_msgs::msg::PointCloud2>("/trans_points",100);
    }

private:

    double rotation_quaternion_x_;
    double rotation_quaternion_y_;
    double rotation_quaternion_z_;
    double rotation_quaternion_w_;
    

    void declare_parameters()
    {   
        declare_parameter<double>("rotation_quaternion_x", 0.0);
        declare_parameter<double>("rotation_quaternion_y", 0.0);
        declare_parameter<double>("rotation_quaternion_z", 0.0);
        declare_parameter<double>("rotation_quaternion_w", 1.0);
    }

    void get_parameters()
    {   
        get_parameter("rotation_quaternion_x", rotation_quaternion_x_);
        get_parameter("rotation_quaternion_y", rotation_quaternion_y_);
        get_parameter("rotation_quaternion_z", rotation_quaternion_z_);
        get_parameter("rotation_quaternion_w", rotation_quaternion_w_);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);
    return cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr applyRotation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        Eigen::Quaternionf q(rotation_quaternion_w_, rotation_quaternion_x_, rotation_quaternion_y_, rotation_quaternion_z_);

        // 변환 행렬 생성
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(q);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform);

        return transformed_cloud;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // Apply rotation transformation
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed = applyRotation(cloud);

        sensor_msgs::msg::PointCloud2 transformed_output;
        pcl::toROSMsg(*cloud_transformed, transformed_output);
        pub_transformed_->publish(transformed_output);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_transformed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCL_Transform>());
    rclcpp::shutdown();
    return 0;
}
