from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, TextSubstitution, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    urdf_path = '/home/vmware/ros_ws/src/mobile_robotics_update/urdf/Security_Robot_Mark_20.xacro'
    
    pkg_share = get_package_share_directory('mobile_robotics_update')
    rviz_path = PathJoinSubstitution([pkg_share, 'config', 'config.rviz'])
    carto_config_dir = os.path.join(pkg_share, 'config')
    carto_config_file = 'Cartographer.lua'

    # Odometry Node (encoder to odom)
    ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[os.path.join(pkg_share, 'config/ekf.yaml')],
    remappings=[('odometry/filtered', 'odom')]
)

    # LIDAR Node (RPLIDAR)
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
            'scan_mode': 'Standard',
            'scan_frequency': 10.0,
            'auto_stanby': False
        }]
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'use_sim_time': False}]
    )

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            '-configuration_directory', carto_config_dir,
            '-configuration_basename', carto_config_file
        ]
    )
    # sub node 
    node_rb = Node(
        package = 'mobile_robotics_update',
        executable = 'joint_nodeRB',
        name='rb_node',
        output='screen'
    )
    node_odom = Node(
        package ='mobile_robotics_update',
        executable= 'odometryRB',
        name='odom_rb',
        output = 'screen'
    )
    lifecycle_manager = ExecuteProcess(
    cmd=[
        'ros2', 'lifecycle', 'set', '/map_server', 'configure',
        '&&',
        'ros2', 'lifecycle', 'set', '/map_server', 'activate',
        '&&',
        'ros2', 'lifecycle', 'set', '/amcl', 'configure',
        '&&',
        'ros2', 'lifecycle', 'set', '/amcl', 'activate'
    ],
    shell=True
)
    # map file
    map_file_path = os.path.join(
        get_package_share_directory('mobile_robotics_update'),
        'map',
        'map_testkub1.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])

    return LaunchDescription([
        # Robot URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': ParameterValue(
                    Command([TextSubstitution(text='xacro '), urdf_path]),
                    value_type=str
                )
            }]
        ),

        # Joint States (จาก encoder)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': ['/joint_states'],
                'qos_overrides./joint_states.subscriber.reliability': 'best_effort',
            }]
        ),

        # RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            parameters=[{
                'use_sim_time': False,
                'qos_overrides./map.subscriber.reliability': 'reliable',
                'qos_overrides./map.subscriber.durability': 'transient_local'
            }]
        ),
        lidar_node,
        node_rb,
        node_odom,
        amcl_node,
        map_server_cmd,
        lifecycle_manager,
        # Launch Components
        # odom_node,
        
        # cartographer_node
        
    ])
