import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    xacro_urdf_file_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_cam_bringup'),
        'urdf',
        'vqwbot.cam.urdf.xacro'
    ])

    robot_description =  {'robot_description': 
                            launch_ros.parameter_descriptions.ParameterValue(
                                Command(['xacro ', xacro_urdf_file_path])
                                , value_type=str)
                                }


    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_cam_bringup"),
        "params",
        "pan_tilt_controllers.yaml"
    ])

    # remappings = [
    #                 ('/diffbot_base_controller/cmd_vel', '/cmd_vel'),
    #               ]

    ld = LaunchDescription()
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'))

    # Publish the "Cam" version of the robot_description for use by the "Cam".ros2_control_node ONLY
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
        namespace="cam",
    )
    ld.add_action(robot_state_pub_node)

    #---------- controller_manager -----------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #arguments=["--ros-args", "--log-level", "DEBUG"],
        parameters=[robot_controllers_path],
        remappings=[('robot_description', '/cam/robot_description')],
        output="both",
    )
    ld.add_action(ros2_control_node)

    #--------- joint_state_broadcaster -------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    #--------- position_controller -------------------
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="CamServoController",
        arguments=["position_controller", "--param-file", robot_controllers_path],
    )
    ld.add_action(robot_controller_spawner)


    return ld


