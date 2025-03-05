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
# # # # # camera calibration URL: file:///home/whit/.ros/camera_info/imx708__base_axi_pcie_120000_rp1_i2c_88000_imx708_1a_640x480.yaml
# # # # #
# # # # #

def generate_launch_description():
    ld = LaunchDescription()

    # # # # # camara_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    # # # # #     [os.path.join(get_package_share_directory('camera_ros'), 'camera.launch.py')])
    # # # # # )
    # # # # # ld.add_action(camara_launch)

    # parameters
    ##---- the Left Camera device name or number
    camera_L_param_name = "camera_L"
    camera_L_param_default = str(0)
    camera_L_param = LaunchConfiguration(
        camera_L_param_name,
        default=camera_L_param_default,
    )
    camera_L_launch_arg = DeclareLaunchArgument(
        camera_L_param_name,
        default_value=camera_L_param_default,
        description="camera_L number or device name"
    )
    ld.add_action(camera_L_launch_arg)

    
    ##---- the Right Camera device name or number
    camera_R_param_name = "camera_R"
    camera_R_param_default = str(0)
    camera_R_param = LaunchConfiguration(
        camera_R_param_name,
        default=camera_R_param_default,
    )
    camera_R_launch_arg = DeclareLaunchArgument(
        camera_R_param_name,
        default_value=camera_R_param_default,
        description="camera_R number or device name"
    )
    ld.add_action(camera_R_launch_arg)

    
    
    
    ##-----------------------------------------
    camera_L_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_L',
            parameters=[{
                "camera": camera_L_param,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": 'XRGB8888',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    ##-----------------------------------------
    camera_R_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_R',
            parameters=[{
                "camera": camera_R_param,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": 'XRGB8888',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    
# /camera_L/camera_info
# /camera_L/image_raw
# /camera_L/image_raw/compressed    
    
    rectify_L_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_L',
            remappings=[
                ('image', '/carera_L/image_raw'),
                #('image_rect', '/carera_L/image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    
    rectify_R_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_R',
            remappings=[
                ('image', '/carera_R/image_raw'),
                #('image_rect', '/carera_R/image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )


    # camera node
    composable_nodes_L = [
        camera_L_composable_node,
        #rectify_L_composable_node,
    ]

    composable_nodes_R = [
        camera_R_composable_node,
        #rectify_R_composable_node,
    ]

    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_container_L',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_L,
    )
    ld.add_action(container)


    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_container_R',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes_R,
    )
    ld.add_action(container)

    return ld


