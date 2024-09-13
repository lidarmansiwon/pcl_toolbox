#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

class Pcl_Tool_Box : public rclcpp::Node
{
public:
    Pcl_Tool_Box()
        : Node("Pcl_Tool_Box")
    {
        // Declare parameters with default values
        declare_parameters();

        // Get parameter values
        get_parameters();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // 센서 데이터에 대한 신뢰할 수 있는 QoS 설정

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/agent3/points", rclcpp::SensorDataQoS(), std::bind(&Pcl_Tool_Box::lidar_callback, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tcvf_points", 100);
        pub_downsampled_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tcv_points", 100);
        pub_transformed_ = create_publisher<sensor_msgs::msg::PointCloud2>("/t_points",100);
    }

private:

    double crop_box_z_min_;
    double crop_box_z_max_;
    double crop_box_x_min_;
    double crop_box_x_max_;
    double crop_box_y_min_;
    double crop_box_y_max_;
    double voxel_resolution_;
    double setMean_;
    double setStddevMulThresh_;
    // double rotation_angle_degrees_;
    // double rotation_angle_radians_;
    double rotation_quaternion_x_;
    double rotation_quaternion_y_;
    double rotation_quaternion_z_;
    double rotation_quaternion_w_;
    

    void declare_parameters()
    {   
        declare_parameter<double>("crop_box_y_min", -10.0);
        declare_parameter<double>("crop_box_y_max", 10.0);
        declare_parameter<double>("crop_box_z_min", -1.0);
        declare_parameter<double>("crop_box_z_max", 1.5);
        declare_parameter<double>("crop_box_x_min", -10.0);
        declare_parameter<double>("crop_box_x_max", 20.0);

        declare_parameter<double>("voxel_resolution", 0.5);
        declare_parameter<double>("setMean", 10.0);
        declare_parameter<double>("setStddevMulThresh", 1.0);
        // declare_parameter<double>("rotation_angle_degrees", 45.0);  // 회전 각도 (도 단위)
        declare_parameter<double>("rotation_quaternion_x", 0.0);
        declare_parameter<double>("rotation_quaternion_y", 0.0);
        declare_parameter<double>("rotation_quaternion_z", 0.0);
        declare_parameter<double>("rotation_quaternion_w", 1.0);
    }

    void get_parameters()
    {   
        get_parameter("crop_box_y_min", crop_box_y_min_);
        get_parameter("crop_box_y_max", crop_box_y_max_);
        get_parameter("crop_box_z_min", crop_box_z_min_);
        get_parameter("crop_box_z_max", crop_box_z_max_);
        get_parameter("crop_box_x_min", crop_box_x_min_);
        get_parameter("crop_box_x_max", crop_box_x_max_);

        get_parameter("voxel_resolution", voxel_resolution_);
        get_parameter("setMean", setMean_);
        get_parameter("setStddevMulThresh", setStddevMulThresh_);
        // get_parameter("rotation_angle_degrees", rotation_angle_degrees_);
        get_parameter("rotation_quaternion_x", rotation_quaternion_x_);
        get_parameter("rotation_quaternion_y", rotation_quaternion_y_);
        get_parameter("rotation_quaternion_z", rotation_quaternion_z_);
        get_parameter("rotation_quaternion_w", rotation_quaternion_w_);

        // 도 단위를 라디안 단위로 변환
        // rotation_angle_radians_ = rotation_angle_degrees_ * M_PI / 180.0;    
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

    // pcl::PointCloud<pcl::PointXYZI>::Ptr applyRotation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    // {
    //     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        
    //     transform.rotate(Eigen::AngleAxisf(rotation_angle_radians_, Eigen::Vector3f::UnitY()));
        
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::transformPointCloud(*input_cloud, *transformed_cloud,transform);
    //     return transformed_cloud;
    // }
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
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // pcl::fromROSMsg(*input, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = convertPointCloud2ToPCL(input);

        // Apply rotation transformation
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed = applyRotation(cloud);

        sensor_msgs::msg::PointCloud2 transformed_output;
        pcl::toROSMsg(*cloud_transformed, transformed_output);
        pub_transformed_->publish(transformed_output);

        // CropBox filter
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped = applyCropBoxFilter(cloud_transformed);

        // VoxelGrid downsampling
        pcl::VoxelGrid<pcl::PointXYZI> sor_downsample;
        sor_downsample.setInputCloud(cloud_cropped);
        sor_downsample.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_); // Set the voxel size
        sor_downsample.filter(*cloud_downsampled);
        // Publish the Data
        sensor_msgs::msg::PointCloud2 downsampled;
        pcl::toROSMsg(*cloud_downsampled, downsampled);
        pub_downsampled_->publish(downsampled);

        // RCLCPP_INFO(this->get_logger(), "Cloud before filtering: %lu points", cloud_downsampled->size());

        // StatisticalOutlierRemoval filtering
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
        sor_filter.setInputCloud(cloud_downsampled);
        sor_filter.setMeanK(setMean_);
        sor_filter.setStddevMulThresh(setStddevMulThresh_);
        sor_filter.filter(*cloud_filtered);

        // Publish the Data
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        pub_->publish(output);

        // RCLCPP_INFO(this->get_logger(), "Cloud after filtering: %lu points", cloud_filtered->size());


    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_downsampled_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_transformed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Pcl_Tool_Box>());
    rclcpp::shutdown();
    return 0;
}
