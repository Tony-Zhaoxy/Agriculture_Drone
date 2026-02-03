from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    serial_no = LaunchConfiguration("serial_no")

    # Streams
    enable_color = LaunchConfiguration("enable_color")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_gyro  = LaunchConfiguration("enable_gyro")
    enable_accel = LaunchConfiguration("enable_accel")

    # Common tuning
    color_width  = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps    = LaunchConfiguration("color_fps")

    depth_width  = LaunchConfiguration("depth_width")
    depth_height = LaunchConfiguration("depth_height")
    depth_fps    = LaunchConfiguration("depth_fps")

    # IMU handling (important for VIO)
    imu_unite_method = LaunchConfiguration("imu_unite_method")
    sync_frames = LaunchConfiguration("sync_frames")      # timestamps alignment
    enable_sync = LaunchConfiguration("enable_sync")      # sync streams inside driver

    # Depth post-processing (optional)
    align_depth = LaunchConfiguration("align_depth")

    declare_args = [
        DeclareLaunchArgument("camera_name", default_value="d455"),
        DeclareLaunchArgument("serial_no", default_value=""),  # leave empty if only one cam

        # If you're doing VIO first, depth can be disabled to save bandwidth
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="false"),  # << default off
        DeclareLaunchArgument("enable_gyro",  default_value="true"),
        DeclareLaunchArgument("enable_accel", default_value="true"),

        # VIO-friendly defaults (safe)
        DeclareLaunchArgument("color_width",  default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps",    default_value="30"),

        # Depth params only matter if enable_depth=true
        DeclareLaunchArgument("depth_width",  default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="480"),
        DeclareLaunchArgument("depth_fps",    default_value="30"),

        # IMU unite: 0/1/2 depending on driver version; 2 is commonly best
        DeclareLaunchArgument("imu_unite_method", default_value="2"),

        # These help keep timestamps sane for VIO pipelines
        DeclareLaunchArgument("enable_sync", default_value="true"),
        DeclareLaunchArgument("sync_frames", default_value="true"),

        # Only meaningful when depth is enabled
        DeclareLaunchArgument("align_depth", default_value="false"),
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

            "color_width":  color_width,
            "color_height": color_height,
            "color_fps":    color_fps,

            "depth_width":  depth_width,
            "depth_height": depth_height,
            "depth_fps":    depth_fps,

            "imu_unite_method": imu_unite_method,
            "enable_sync": enable_sync,
            "sync_frames": sync_frames,

            "align_depth.enable": align_depth,

            # Small quality-of-life settings:
            "publish_tf": True,
            "tf_publish_rate": 50.0,
        }],
    )

    return LaunchDescription(declare_args + [realsense_node])
