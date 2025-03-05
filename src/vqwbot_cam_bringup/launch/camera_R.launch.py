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
    ##---- the camera device name or number
    camera_param_name = "camera"
    camera_param_default = str(1)
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
    

    ##-----------------------------------------
    camera_composable_node = ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera_R',
            parameters=[{
                "camera": camera_param,
                "width": 1040,  #640,
                "height": 768,  #480,
                "format": format_param,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    
    rectify_composable_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_R',
            remappings=[
                ('image', '/camera_L/image_raw'),
                #('image_rect', 'image_rect')
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )


    # camera node
    composable_nodes = [
        camera_composable_node,
        #rectify_composable_node,
    ]

    # composable nodes in single container
    container = ComposableNodeContainer(
        name='camera_R_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )
    ld.add_action(container)

    return ld


