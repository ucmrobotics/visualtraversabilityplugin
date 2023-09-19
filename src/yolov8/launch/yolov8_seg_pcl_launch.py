#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

import os

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_share = FindPackageShare(package='husky_master').find('husky_master')

    yolov8_segment_node = Node(
        package='yolov8',
        executable='yolov8_segment_node',
        parameters=[
            {"model_path": "/home/ettore/AgBot/src/husky_vision/yolov8/models/huskynav_yolov8_1280x720.pt"},
            {"device": "cpu"},
            {"frequency": 2.0},
            {"queue_size": 10},
            {"delay_threshold": 0.3},
            {"NN_threshold": 0.25},
            {"enable": True},
            {"debug": True},
        ],
        remappings=[
            ("color/image", "/color/image"),
            ("stereo/depth", "/stereo/converted_depth"),
            ("stereo/camera_info", "/stereo/camera_info"),
            ("image_seg/depth", "/image_seg/depth"),
            ("image_seg/info", "/image_seg/info"),
        ]
    )
    
    depth_metric_converter = ComposableNode(
                            package='depth_image_proc',
                            plugin='depth_image_proc::ConvertMetricNode',
                            name='convert_metric_node',
                            remappings=[('image_raw', 'stereo/depth'),
                                        ('camera_info', 'stereo/camera_info'),
                                        ('image', 'stereo/converted_depth')]
                            )
                            
    point_cloud_xyz = ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ("image_rect", "/image_seg/depth"),
                    ("camera_info", "/image_seg/info"),
                    ("points", "/depth_proc/points"),
                ]
            )

    depth_image_proc_container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            depth_metric_converter,
            point_cloud_xyz,
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(yolov8_segment_node)
    ld.add_action(depth_image_proc_container)

    return ld
