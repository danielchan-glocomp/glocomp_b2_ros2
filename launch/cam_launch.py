import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    share_dir = get_package_share_directory('glocomp_b2_ros2')
    launch_dir = os.path.join(share_dir, 'launch')
    
    front_cam = Node(
        package='glocomp_b2_ros2',
        executable='front_cam',
        name='front_cam',
        output='screen',
        parameters=[]
    )

    back_cam = Node(
        package='glocomp_b2_ros2',
        executable='back_cam',
        name='back_cam',
        output='screen',
        parameters=[]
    )
    
    ld = LaunchDescription()
    ld.add_action(front_cam)
    ld.add_action(back_cam)
    return ld
