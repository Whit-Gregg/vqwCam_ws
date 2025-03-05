from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # servo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('vqwbot_cam_bringup'), 'launch', 'servo.launch.py')])
    # )
    #ld.add_action(servo_launch)

    camara_launch_L = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('vqwbot_cam_bringup'), 'launch', 'camera_L.launch.py')])
        ,launch_arguments={ 
                           'camera': '0',
        }.items()
    )
    ld.add_action(camara_launch_L)


    camara_launch_R = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('vqwbot_cam_bringup'), 'launch', 'camera_R.launch.py')])
        ,launch_arguments={  
                           'camera': '1',
                           }.items()
    )
    ld.add_action(camara_launch_R)


    return ld
