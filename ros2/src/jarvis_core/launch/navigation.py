from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import ThisLaunchFileDir
import os
import launch_ros.actions
import pathlib

parameters_file_name = 'config.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/jarvis_core/' + parameters_file_name
    print(parameters_file_path)
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/core.py']),
            # launch_arguments={'node_name': 'bar'}.items(),
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='odometry',
            name='odometry',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
    ])