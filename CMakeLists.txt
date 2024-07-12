cmake_minimum_required(VERSION 3.5)
project(pcl_toolbox)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(Pcl_Tool_Box src/pcl_tool_box.cpp)
ament_target_dependencies(Pcl_Tool_Box rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(PCL_Transform src/pcl_transform.cpp)
ament_target_dependencies(PCL_Transform rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(PCL_CropBox src/pcl_cropbox.cpp)
ament_target_dependencies(PCL_CropBox rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(PCL_Voxelize src/pcl_voxelize.cpp)
ament_target_dependencies(PCL_Voxelize rclcpp sensor_msgs pcl_conversions pcl_ros)

add_executable(PCL_SOR_Filter src/pcl_sor_filter.cpp)
ament_target_dependencies(PCL_SOR_Filter rclcpp sensor_msgs pcl_conversions pcl_ros)

install(TARGETS
  Pcl_Tool_Box
  PCL_Transform
  PCL_CropBox
  PCL_Voxelize
  PCL_SOR_Filter
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
