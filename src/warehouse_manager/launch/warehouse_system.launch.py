# MTRX3760 2025 Project 2: Warehouse Robot DevKit
# File: warehouse_system.launch.py
# Author(s): max rogers
#
# Starts the WarehouseManager node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_manager',
            executable='warehouse_manager_node',
            name='warehouse_manager',
            output='screen',
            parameters=[
                {'log_dir': '/home/george-anastasiadis/turtlebot3_ws/logs'},         # where the folders are saved to - edit for your computer
                {'map_dir': '/home/george-anastasiadis/turtlebot3_ws/maps'},
                {'use_sim_time': True}
            ]
        )
    ])
