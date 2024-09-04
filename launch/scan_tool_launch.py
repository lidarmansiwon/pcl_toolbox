from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,RegisterEventHandler
import launch
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    pcl_topic = LaunchConfiguration("pcl_topic")

    filter_node = Node(

            package="pcl_toolbox",
            executable="ScanTool",
            name="ScanTool",
            output="screen",
            emulate_tty=True,
            remappings=[('/ouster/points', pcl_topic)],
            parameters=[
                {"crop_box_x_min": -3.0},
                {"crop_box_x_max": 20.0},
                {"crop_box_y_min": -5.0},
                {"crop_box_y_max": 5.0},
                {"crop_box_z_min": -0.0},
                {"crop_box_z_max": 0.3},
                {"setMean": 10.0},
                {"setStddevMulThresh": 30.0},
            ]
        )
    

    return LaunchDescription([
        DeclareLaunchArgument("pcl_topic", description="a pointcloud topic to process", default_value="ouster/points"),
        filter_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=filter_node,
                on_start=[launch.actions.LogInfo(msg=[f"\033[92mPCL_Tool_Box Node has started.\033[0m"])]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=filter_node,
                on_exit=[launch.actions.LogInfo(msg=[f"\033[92mPCL_Tool_Box Node has exited.\033[0m"])]
            )
        ),
        launch.actions.OpaqueFunction(function=lambda context: [
            launch.actions.LogInfo(msg=[f"\033[92mSelected pcl_Topic : {pcl_topic.perform(context)}\033[0m"]),
            launch.actions.LogInfo(msg=[f"\033[92mSelected Output_Topic : /tcvf_points\033[0m"])
        ]),
    ])
