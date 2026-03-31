import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition  # Импортируем условия


def generate_launch_description():
    ld = launch.LaunchDescription()
    pkg_name = "monodepth_navigation"
    venv_path = os.path.expanduser('~/ros2_workspace/src/monodepth_navigation/python-env/bin/python3')


    uav_name = LaunchConfiguration('uav_name')
    input_topic = LaunchConfiguration('input_topic')
    input_camera_info_topic = LaunchConfiguration('input_camera_info_topic')
    output_topic = LaunchConfiguration('output_topic')
    distortion_coeffs = LaunchConfiguration('distortion_coeffs')

    use_custom = LaunchConfiguration('use_custom')


    ld.add_action(DeclareLaunchArgument(
        'uav_name', default_value=os.getenv('UAV_NAME', "uav1")
    ))
    ld.add_action(DeclareLaunchArgument(
        'input_topic', default_value='rgb/image_raw'
    ))
    ld.add_action(DeclareLaunchArgument(
        'input_camera_info_topic', default_value='rgb/camera_info'
    ))
    ld.add_action(DeclareLaunchArgument(
        'output_topic', default_value='rgb/image_undistorted'
    ))
    ld.add_action(DeclareLaunchArgument(
        'distortion_coeffs', default_value='[-0.03, 0.0, 0.0, 0.0, 0.0]'
    ))

    ld.add_action(DeclareLaunchArgument(
        'use_custom', default_value='false',
        description="If 'true', runs Python script. If 'false', runs image_proc."
    ))


    custom_node = Node(
        condition=IfCondition(use_custom),
        package=pkg_name,
        namespace=uav_name,
        executable='image_undistorter.py',
        name='image_undistorter',
        prefix=[venv_path + ' '],
        parameters=[
            {'use_sim_time': True},
            {'input_topic': input_topic},
            {'input_camera_info_topic': input_camera_info_topic},
            {'output_topic': output_topic},
            {'distortion_coeffs': distortion_coeffs},
        ],
        output='screen'
    )

    standard_node = Node(
        condition=UnlessCondition(use_custom),
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        namespace=uav_name,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('image', input_topic),
            ('camera_info', input_camera_info_topic),
            ('image_rect', output_topic)
        ],
        output='screen'
    )

    ld.add_action(custom_node)
    ld.add_action(standard_node)

    return ld