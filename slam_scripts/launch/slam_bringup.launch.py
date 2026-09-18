import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_scripts_share = get_package_share_directory('slam_scripts')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    default_params_file = os.path.join(slam_scripts_share, 'config', 'go2_slam.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock published by Gazebo',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the slam_toolbox parameters file',
    )

    cloud_to_scan_node = Node(
        package='slam_scripts',
        executable='cloud_to_scan',
        name='cloud_to_scan_py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    odom_to_tf_node = Node(
        package='slam_scripts',
        executable='odom_to_tf',
        name='odom_to_footprint_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        cloud_to_scan_node,
        odom_to_tf_node,
        slam_toolbox_launch,
    ])
