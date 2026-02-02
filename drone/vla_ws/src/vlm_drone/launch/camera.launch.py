from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Basic args
    camera_name = LaunchConfiguration("camera_name")
    serial_no = LaunchConfiguration("serial_no")
    enable_color = LaunchConfiguration("enable_color")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_gyro = LaunchConfiguration("enable_gyro")
    enable_accel = LaunchConfiguration("enable_accel")
    imu_unite_method = LaunchConfiguration("imu_unite_method")
    align_depth = LaunchConfiguration("align_depth")

    # Recommended defaults for D455
    declare_args = [
        DeclareLaunchArgument("camera_name", default_value="d455"),
        DeclareLaunchArgument("serial_no", default_value=""),  # leave empty to auto-pick
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_gyro", default_value="true"),
        DeclareLaunchArgument("enable_accel", default_value="true"),
        # IMU sync/aggregation method; 2 is common in realsense2_camera
        DeclareLaunchArgument("imu_unite_method", default_value="2"),
        DeclareLaunchArgument("align_depth", default_value="true"),
    ]

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        output="screen",
        parameters=[{
            "camera_name": camera_name,
            "serial_no": serial_no,
            "enable_color": enable_color,
            "enable_depth": enable_depth,
            "enable_gyro": enable_gyro,
            "enable_accel": enable_accel,
            "imu_unite_method": imu_unite_method,
            "align_depth.enable": align_depth,

            # Optional: set resolutions/fps (you can tune later)
            "color_width": 640,
            "color_height": 480,
            "color_fps": 30,
            "depth_width": 640,
            "depth_height": 480,
            "depth_fps": 30,
        }],
    )

    return LaunchDescription(declare_args + [realsense_node])
