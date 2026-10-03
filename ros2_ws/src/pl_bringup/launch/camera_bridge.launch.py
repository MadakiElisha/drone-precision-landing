import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_yolo = LaunchConfiguration("use_yolo", default="true")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        arguments=[
            "/downward_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/downward_camera", "/pl/camera/image_raw"),
            ("/camera_info", "/pl/camera/camera_info"),
        ],
        output="screen",
    )
    aruco = Node(
        package="pl_perception",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        condition=UnlessCondition(use_yolo),
    )
    yolo = Node(
        package="pl_perception",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        condition=IfCondition(use_yolo),
    )
    estimator = Node(
        package="pl_perception",
        executable="tag_estimator",
        name="tag_estimator",
        output="screen",
    )
    control = Node(
        package="pl_control",
        executable="landing_controller",
        name="landing_controller",
        parameters=[{"allow_descent": True},
                    {"tags_topic": "/pl/perception/tags_est"},
                    {"waypoints": os.environ.get("PL_WAYPOINTS", "")},
                    {"spawn": os.environ.get("PL_SPAWN", "0,0")}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_yolo", default_value="true"),
        bridge, aruco, yolo, estimator, control,
    ])
