from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        arguments=[
            "/downward_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/downward_camera", "/pl/camera/image_raw"),
            ("/camera_info", "/pl/camera/camera_info"),
        ],
        output="screen",
    )
    return LaunchDescription([bridge])
