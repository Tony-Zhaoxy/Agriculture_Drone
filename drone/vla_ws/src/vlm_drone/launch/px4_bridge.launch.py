from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    transport = LaunchConfiguration("transport")     # udp4 or serial
    udp_port = LaunchConfiguration("udp_port")       # e.g. 8888
    serial_dev = LaunchConfiguration("serial_dev")   # e.g. /dev/ttyUSB0
    serial_baud = LaunchConfiguration("serial_baud") # e.g. 921600

    declare_args = [
        DeclareLaunchArgument("transport", default_value="udp4"),
        DeclareLaunchArgument("udp_port", default_value="8888"),
        DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baud", default_value="921600"),
    ]

    # We choose command at runtime via shell expansion
    # If transport=udp4 -> MicroXRCEAgent udp4 -p <port>
    # If transport=serial -> MicroXRCEAgent serial --dev <dev> -b <baud>
    agent_cmd = [
        "bash", "-lc",
        "if [ \"" + str(transport) + "\" = \"serial\" ]; then "
        "  echo '[PX4] MicroXRCEAgent serial...'; "
        "  MicroXRCEAgent serial --dev " + str(serial_dev) + " -b " + str(serial_baud) + "; "
        "else "
        "  echo '[PX4] MicroXRCEAgent udp4...'; "
        "  MicroXRCEAgent udp4 -p " + str(udp_port) + "; "
        "fi"
    ]

    agent = ExecuteProcess(
        cmd=agent_cmd,
        output="screen"
    )

    return LaunchDescription(declare_args + [agent])
