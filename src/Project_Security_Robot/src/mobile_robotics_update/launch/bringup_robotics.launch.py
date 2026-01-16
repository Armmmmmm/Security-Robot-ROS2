import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    my_robot_pkg_dir = get_package_share_directory('mobile_robotics_update')
    
    urdf_path = os.path.join(my_robot_pkg_dir, 'urdf', 'Security_Robot_Mark_20.xacro')
    ekf_config_path = os.path.join(my_robot_pkg_dir, 'config', 'ekf.yaml')
    
    joy_config_path = os.path.join(my_robot_pkg_dir, 'config', 'joystick.yaml')

    # --- Robot Description & State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
        }]
    )

    # --- Hardware & Odometry Nodes ---
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable ='micro_ros_agent',
        name='micro_ros_agent',
        output= 'screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200']
    )
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,   
            'scan_mode': 'Standard'
        }]
    )
    joint_node = Node(
        package='mobile_robotics_update',
        executable='joint_nodeRB',
        name='joint_state_publisher_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
    odom_node = Node(
        package='mobile_robotics_update',
        executable='odometryRB',
        name='odometry_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'orientation_stddev': 0.05
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data')
        ]
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}],
        remappings=[('odometry/filtered', 'odom')]
    )
    
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[joy_config_path]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_config_path],
        remappings=[('/cmd_vel_joy', '/cmd_vel')]
    )

    # --- สร้างและคืนค่า Launch Description ---
    return LaunchDescription([
        robot_state_publisher_node,
        lidar_node,
        micro_ros_node,
        joint_node,
        odom_node,
        imu_filter_node,
        ekf_node,
        
        # --- Joystick nodes are removed from the launch list ---
        joy_node,
        teleop_twist_joy_node,
    ])