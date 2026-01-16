import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    
    package_name='mobile_robotics' #<--- CHANGE ME


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','view_robot.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'circle_bot'],
                        output='screen')

    joint_state_broad = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])    

    diff_controller = Node(package='controller_manager' , executable='spawner', arguments=['diff_drive_cont'])
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_controller,
         TimerAction(
        period=5.0,
        actions=[Node(package='controller_manager', executable='spawner', arguments=['diff_drive_cont', '--ros-args', '--params-file', '/home/arm/ros2_ws/src/mobile_robotics/config/controllers.yaml'])]),
        joint_state_broad
    ])