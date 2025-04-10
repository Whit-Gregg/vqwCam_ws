import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    ##---- the camera image format ---
    format_param_name = "format"
    format_param_default = "XRGB8888"
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )
    ld.add_action(format_launch_arg)
    


    ##-----------------------------------------
    camera_L_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_L',
            parameters=[{
                "camera": 0,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
            ## output published to: /carera_L/image_raw
        )

    ##-----------------------------------------
    camera_R_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_R',
            parameters=[{
                "camera": 1,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
            ## output published to: /carera_L/image_raw
        )
    
    ##-----------------------------------------
    rectify_L_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_L',
            remappings=[
                ('image', '/carera_L/image_raw'),
                ('image_rect', '/carera_L/image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ## output to:  /carera_L/image_rect
        )
    
    
    ##-----------------------------------------
    rectify_R_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_R',
            remappings=[
                ('image', '/carera_R/image_raw'),
                ('image_rect', '/carera_R/image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ## output to:  /carera_L/image_rect
        )
    


    # camera node
    composable_nodes_part_A = [
        camera_L_composable_node,
        #camera_R_composable_node,
        rectify_L_composable_node,
        rectify_R_composable_node,
    ]

    # camera node
    composable_nodes_part_B = [
        #camera_L_composable_node,
        camera_R_composable_node,
        # rectify_L_composable_node,
        # rectify_R_composable_node,
    ]

    # composable nodes in single container
    container_B = ComposableNodeContainer(
        name='camera_Small_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_part_B,
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    ld.add_action(container_B)


    # composable nodes in single container
    container_A = ComposableNodeContainer(
        name='camera_Big_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_part_A,
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    ld.add_action(container_A)


    stereo_image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('stereo_image_proc'), 'launch', 'stereo_image_proc.launch.py')])
        ,launch_arguments={  
                           'approximate_sync': 'True',
                           'avoid_point_cloud_padding': 'True',
                           'left_namespace': 'camera_L',
                           'right_namespace': 'camera_R',
                           'launch_image_proc': 'True',
                           'container': 'camera_Big_container',
                           }.items()
    )
    ld.add_action(stereo_image_proc_launch)

    return ld
