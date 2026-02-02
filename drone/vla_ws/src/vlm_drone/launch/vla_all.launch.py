from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_vlm = get_package_share_directory("vlm_drone")
    px4_launch = os.path.join(pkg_vlm, "launch", "px4_bridge.launch.py")
    cam_launch = os.path.join(pkg_vlm, "launch", "camera.launch.py")

    # IMPORTANT: point to your server.py path in the workspace
    server_py = os.path.expanduser("~/AGRICULTURE_DRONE/drone/vla_ws/src/vla_server/server.py")

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(px4_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cam_launch)),

        # start Flask server as a plain python process
        ExecuteProcess(
            cmd=["python3", server_py],
            output="screen"
        ),

        # start controller node
        Node(
            package="vlm_drone",
            executable="drone_node",
            name="drone_node",
            output="screen",
            parameters=[{"server_url": "http://127.0.0.1:5000/predict_action"}],
        ),
    ])
