import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('gazebo_bringup').find('gazebo_bringup')

    # Files paths
    default_world_path = os.path.join(pkg_share, 'worlds/six_waypoints.world')
    rviz_config = os.path.join(pkg_share, 'rviz/config.rviz')

    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')
    use_rviz = LaunchConfiguration('use_rviz') 
    
    declare_arguments = [
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),

        DeclareLaunchArgument(  
            name='verbose',
            default_value='false',
            description='Set "true" to increase messages written to terminal.'
        ),

        DeclareLaunchArgument(  
            name='use_rviz',
            default_value='false',
            description='To open rviz tool'
        ),
    ]
    
    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world'     : world,
            'verbose'   : verbose
            }.items()
    )

    # Spawn robot in simulation environment
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'robot_name'    : 'questbot_2wd',
            'spawn_x'       : '0.0',
            'spawn_y'       : '0.0',
            'spawn_z'       : '0.4',
            'spawn_yaw'     : '0.0'
        }.items()
    )
    
    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
        condition=IfCondition(use_rviz)
    )


    return LaunchDescription(
        declare_arguments + [
            start_gazebo,
            spawn_robot,
            start_rviz
        ]
    )