import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mr_loco_desc = get_package_share_directory('mr_loco_description')
    world_path= get_package_share_directory("mr_loco_description")+'/worlds/world.sdf'

    # gz_sim = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #             ),
    #             launch_arguments={'gz_args': '-r empty.sdf'}.items()     
    #         )
    
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_ros_gz_sim, '/launch', '/gz_sim.launch.py']),
        launch_arguments={
                    'gz_args' : world_path + " -v 4"
                }.items())
    
    # spawn robot with rviz
    robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_mr_loco_desc, 'launch', 'robot.launch.py')
                )
            )

    return LaunchDescription([
        gazebo,
        robot
    ])
