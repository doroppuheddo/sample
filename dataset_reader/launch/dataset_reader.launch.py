import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # IMU publisher node
    stored_imu_publisher = Node(
        package="dataset_reader",
        executable="stored_imu_publisher",
        name="stored_imu_publisher",
        output="screen",
    )

    # Image publisher node
    stored_image_publisher = Node(
        package="dataset_reader",
        executable="stored_image_publisher",
        name="stored_image_publisher",
        output="screen",
        #parameters=[{"image_path": os.path.join(get_package_share_directory("my_package"), "data", "example_image.png")}],
    )

    # Add the nodes to the launch description
    ld.add_action(stored_imu_publisher)
    ld.add_action(stored_image_publisher)

    return ld
