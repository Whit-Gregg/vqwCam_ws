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

# # # # #
# # # # #
# # # # # camera calibration URL: file:///home/whit/.ros/camera_info/xxxxxxx.yaml
# # # # #
# # # # #

def generate_launch_description():
    ld = LaunchDescription()

    # parameters
    ##---- the camera device name or number
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name,
        default_value=camera_param_default,
        description="camera ID or device name"
    )
    ld.add_action(camera_launch_arg)

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
    
    apriltag_config_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_cam_bringup"),
        "params",
        "apriltag_params.yaml"
    ])
    
    
    ##-----------------------------------------
    camera_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_L',
            parameters=[{
                "camera": camera_param,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
            ## output published to: /carera_L/image_raw
        )
    
    rectify_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_L',
            remappings=[
                ('image', '/carera_L/image_raw'),
                ('image_rect', '/carera_L/image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[{
                'qos_overrides': {
                    '/carera_L/image_rect': {
                        'publisher': {
                            'reliability': 'best_effort',
                            'history': 'keep_last',
                            'depth': 100,
                        }
                    }
                },
            }],
        ## output to:  /carera_L/image_rect
        )
    
    
    apriltag_node = ComposableNode(
            package='apriltag_ros',
            plugin='apriltag::AprilTagNode',
            name='apriltag',
            remappings=[
                ('image_rect', '/carera_L/image_rect'),
            ],
            parameters=apriltag_config_path,
            extra_arguments=[{'use_intra_process_comms': True}],
            ## output published to: /carera_L/image_raw
        )
    


    # camera node
    composable_nodes = [
        camera_composable_node,
        rectify_composable_node,
        apriltag_node,
    ]

    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_L_apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    ld.add_action(container)

    return ld

