#!/usr/bin/env python3
# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode



def generate_launch_description():
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')

    camera_fps = LaunchConfiguration('camera_fps', default='30')
    resolution = LaunchConfiguration('resolution', default='[640, 480]')
    namespace = LaunchConfiguration('namespace', default='')

    ARGUMENTS = [
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30',
            description='Camera frame rate'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='[640, 480]',
            description='Camera resolution'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        )
    ]

    node = ComposableNodeContainer(
            name='oakd_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='depthai_ros_driver',
                        plugin='depthai_ros_driver::Camera',
                        name='oakd',
                        parameters=[{
                            'camera_fps': camera_fps,
                            'resolution': resolution
                        }]
                    ),
            ],
            output='screen',
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node)
    return ld
