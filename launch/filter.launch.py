#!/usr/bin/env/ python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch_ros.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # param_dir = LaunchConfiguration(
    #     'param_dir',
    #     default=os.path.join(
    #     get_package_share_directory('fusion_test'),
    #     'param',
    #     'param_base.yaml'))
    
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'param_dir',
        #     default_value=param_dir,
        #     description='Full path to param file to load'),

        Node(
            package='velodyne_filter',
            executable='velodyne_DownSampling',
            name='velodyne_DownSampling',
            output='screen'),

        Node(
            package='velodyne_filter',
            executable='velodyne_ROI',
            name='velodyne_ROI',
            output='screen'),         

        Node(
            package='velodyne_filter',
            executable='velodyne_cluster',
            name='velodyne_cluster',
            output='screen'),                  
    ])        
        