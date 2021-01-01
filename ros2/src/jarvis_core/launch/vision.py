from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
import os
import launch_ros.actions
import pathlib

parameters_file_name = 'config.yaml'

def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/jarvis_core/' + parameters_file_name
    print(parameters_file_path)
    return LaunchDescription([
        Node(
            package='joy_linux',
            namespace='jarvis',
            executable='joy_linux_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            namespace='jarvis',
            executable='teleop_node',
            name='teleop_node',
            remappings=[
              ('/joy', '/jarvis/joy'),
            ],
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='t2mvel',
            name='t2mvel',
            remappings=[
              ('/cmd_vel', '/jarvis/cmd_vel'),
            ],
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='wv2pwm',
            name='left_wv2pwm',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='wv2pwm',
            name='right_wv2pwm',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='jetty',
            name='jetty',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='stat',
            name='stat',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
        Node(
            package='jarvis_core',
            namespace='jarvis',
            executable='camera',
            name='camera',
            output='screen',
            parameters=[
                parameters_file_path
            ]
        ),
    ])