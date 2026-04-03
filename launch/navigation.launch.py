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

    is_reactive = LaunchConfiguration('is_reactive')
    x_octogoal = LaunchConfiguration('x_octogoal')
    y_octogoal = LaunchConfiguration('y_octogoal')
    z_octogoal = LaunchConfiguration('z_octogoal')
    yaw_octogoal = LaunchConfiguration('yaw_octogoal')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    ld.add_action(DeclareLaunchArgument(
        'is_reactive',
        default_value='false',
        description="Whether to use reactive navigation or octomap planner"
    ))

    ld.add_action(DeclareLaunchArgument('x_octogoal', default_value='5.0', description="X target for Octomap planner"))
    ld.add_action(DeclareLaunchArgument('y_octogoal', default_value='0.0', description="Y target for Octomap planner"))
    ld.add_action(DeclareLaunchArgument('z_octogoal', default_value='2.0', description="Z target for Octomap planner"))
    ld.add_action(
        DeclareLaunchArgument('yaw_octogoal', default_value='0.0', description="Yaw target for Octomap planner"))


    # #} end of custom_config

    # #{ blob detector node

    navigation_node = Node(
        package=pkg_name,
        namespace=uav_name,
        name='navigation_controller',
        executable='navigation_controller.py',
        prefix=[venv_path + ' '],
        parameters=[
                    {'use_sim_time': False},
                    {'is_reactive': is_reactive},
                    {'target_frame': [uav_name, '/fcu_untilted']},
                    {'output_velocity_topic': ['/', uav_name, '/control_manager/velocity_reference']},
                    {'world_frame': [uav_name, '/local_origin']},
                    {'body_frame': [uav_name, '/fcu_untilted']},

                    {'input_pointcloud_topic': '/midas/pointcloud_by_map'},

                    {'x_octogoal': x_octogoal},
                    {'y_octogoal': y_octogoal},
                    {'z_octogoal': z_octogoal},
                    {'yaw_octogoal': yaw_octogoal},
                ]
    )

    # #} end of sweeping generator node

    ld.add_action(navigation_node)

    return ld


















