from setuptools import setup, find_packages
from glob import glob
import os

package_name = "vlm_drone"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Index for ament
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Package manifest
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # (Optional) install scripts
        (os.path.join("share", package_name, "script"), glob("script/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nuc",
    maintainer_email="nuc@todo.todo",
    description="VLA drone controller (PX4 ROS2 offboard)",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run vlm_drone drone_node
            "drone_node = vlm_drone.drone_node:main",
        ],
    },
)
