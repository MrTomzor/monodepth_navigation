#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "monodepth_navigation"

    this_pkg_path = get_package_share_directory(pkg_name)

    venv_path = this_pkg_path + "/python-env/bin/python3"

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ blob detector node

    monodepth_node = Node(
        package=pkg_name,
        namespace=uav_name,
        name='monodepth_node',
        executable='monocular_depth_estimator.py',
        prefix=[venv_path + ' '],
        parameters=[
            # {'use_sim_time': False}, # real world
            {'use_sim_time': True}, # simulation
            #
            # {'input_img_topic': '/oak/rgb/image_raw/compressed'},
            # {'input_camera_info_topic': '/oak/rgb/camera_info'},
            # {'input_pointcloud_topic': ['/', uav_name, '/livox/lidar_front/points']},
            # # {'input_pointcloud_topic': '/uav1/open_vins_front/points_slam'},
            # {'camera_frame': 'oak_rgb_camera_frame'},
            # {'optical_camera_frame': 'oak_rgb_camera_optical_frame'},
            # {'world_frame': 'uav91/world_origin'},
            # {'is_camera_inverted': True},

            {'input_img_topic': '/uav1/rgb/image_raw'},
            {'input_camera_info_topic': '/uav1/rgb/camera_info'},
            # {'input_pointcloud_topic': ['/', uav_name, '/lidar/points']},
            {'input_pointcloud_topic': ['/', uav_name, '/open_vins/points_slam']},
            {'camera_frame': 'uav1/rgb'},
            {'world_frame': 'uav1/local_origin'},

            {'output_depth_map_topic': '/midas/depth_view'},
            {'output_scaled_depth_map_topic_map': '/midas/scaled_depth_view_map'},
            {'output_pointcloud_topic_map': '/midas/pointcloud_by_map'},
            {'output_pointcloud_topic_value': '/midas/pointcloud_by_value'},


        ],
        output='screen',

    )

    # #} end of sweeping generator node

    ld.add_action(monodepth_node)

    return ld





