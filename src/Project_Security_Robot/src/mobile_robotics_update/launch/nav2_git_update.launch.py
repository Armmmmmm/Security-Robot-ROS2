import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    ws_dir = get_package_share_directory('mobile_robotics_update')
    default_params_file = os.path.join(ws_dir, "config", "nav2_git_paramskub.yaml")

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file)
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')
    declare_map_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')

    param_substitutions = {
        'map_server.ros__parameters.yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
        
    node_package_map = {
        'controller_server': 'nav2_controller',
        'smoother_server': 'nav2_smoother',
        'planner_server': 'nav2_planner',
        'behavior_server': 'nav2_behaviors',
        'bt_navigator': 'nav2_bt_navigator',
        'waypoint_follower': 'nav2_waypoint_follower',
        'velocity_smoother': 'nav2_velocity_smoother',
        'map_server': 'nav2_map_server',
        'amcl': 'nav2_amcl'
    }

    lifecycle_nodes = list(node_package_map.keys())

    load_nodes = [
        LifecycleNode(
            package=package_name, executable=node_name, name=node_name,
            namespace='', output='screen', 
            parameters=[configured_params, {'use_sim_time': use_sim_time}]
        ) for node_name, package_name in node_package_map.items()
    ]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )
    
    rviz2_config_file = LaunchConfiguration('rviz_config')
    declare_rviz2_cmd = DeclareLaunchArgument('rviz_config', default_value=os.path.join(ws_dir, 'config', 'navigationRviz_.rviz'))
    start_rviz_cmd = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz2_config_file], output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_rviz2_cmd)
    
    for node in load_nodes:
        ld.add_action(node)
        
    ld.add_action(lifecycle_manager_node)
    ld.add_action(start_rviz_cmd)
    
    return ld