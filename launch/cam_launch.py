import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    share_dir = get_package_share_directory('glocomp_b2_ws')
    launch_dir = os.path.join(share_dir, 'launch')
    
    front_cam = Node(
        package='glocomp_b2_ws',
        executable='front_cam',
        name='front_cam',
        output='screen',
        parameters=[]
    )

    back_cam = Node(
        package='glocomp_b2_ws',
        executable='back_cam',
        name='back_cam',
        output='screen',
        parameters=[]
    )

    cmd_vel = Node(
        package='glocomp_b2_ws',
        executable='cmd_vel_subscriber',
        name='cmd_vel',
        output='screen',
        parameters=[]
    )


    
    ld = LaunchDescription()
    ld.add_action(front_cam)
    ld.add_action(back_cam)
    ld.add_action(cmd_vel)
    return ld
