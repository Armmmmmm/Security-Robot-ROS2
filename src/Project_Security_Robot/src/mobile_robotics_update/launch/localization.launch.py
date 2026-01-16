import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # --- Paths ---
    my_robot_pkg_dir = get_package_share_directory('mobile_robotics_update')
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')

    # --- File Paths ---
    # Path to your robot's main launch file
    robot_bringup_launch_path = os.path.join(my_robot_pkg_dir, 'launch', 'robot_bringup_launch.py')
    
    # Path to your localization parameters
    localization_params_path = os.path.join(my_robot_pkg_dir, 'config', 'localization_real_robot.yaml')

    # Path to the slam_toolbox launch file we want to use
    slam_launch_path = os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')

    # Path to your RViz configuration
    rviz_config_path = os.path.join(my_robot_pkg_dir, 'config', 'localization.rviz')


    # --- 1. Launch the robot's base functionality ---
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch_path)
    )

    # --- 2. Launch SLAM Toolbox for localization ---
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': localization_params_path
        }.items()
    )

    # --- 3. Launch RViz to visualize everything ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    return LaunchDescription([
        robot_bringup_launch,
        slam_toolbox_launch,
        rviz_node
    ])