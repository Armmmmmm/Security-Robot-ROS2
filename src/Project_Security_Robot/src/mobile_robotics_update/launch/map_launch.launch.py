import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Paths and Default Files ---
    ws_dir = get_package_share_directory('mobile_robotics_update')
    default_map_file = os.path.join(ws_dir, 'map', 'map_testkub1.yaml')
    default_rviz_config = os.path.join(ws_dir, 'config', 'map_rviz.rviz') 
    default_params_file = os.path.join(ws_dir, 'config', 'nav2_git_paramskub.yaml')

    # --- 2. Launch Configurations ---
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # --- 3. Declare Launch Arguments ---
    declare_map_cmd = DeclareLaunchArgument('map', default_value=default_map_file)
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_rviz_config_cmd = DeclareLaunchArgument('rviz_config', default_value=default_rviz_config)
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file)
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')

    # --- 4. Define Nodes ---

    # Node 1: RViz (จะถูกเรียกเป็นลำดับแรกสุด)
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen'
    )

    # Node 2: Map Server
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_file}]
    )

    # Node 3: AMCL
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params_file, 
                    {'use_sim_time': use_sim_time}]
    )

    # Node 4: Lifecycle Manager
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # --- 5. Assemble the Launch Description ---
    ld = LaunchDescription()

    # เพิ่มการประกาศ Argument
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # --- การเปลี่ยนแปลงที่สำคัญ: เรียงลำดับใหม่ ---
    # 1. รัน RViz ก่อน
    ld.add_action(rviz_node)
    
    # 2. จากนั้นค่อยรันส่วนของ Localization
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_localization)
    
    return ld