import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- Path Definitions (เหมือนเดิม) ---
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ws_dir = get_package_share_directory('mobile_robotics_update')
    
    # --- Default file paths (เหมือนเดิม) ---
    default_params_file = os.path.join(ws_dir, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(ws_dir, 'map', 'my_map_serial.yaml')
    default_rviz_config = os.path.join(ws_dir, 'rviz', 'nav2_.rviz')

    # --- Launch configurations (เหมือนเดิม) ---
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz_config = LaunchConfiguration('rviz_config')

    # --- Declare launch arguments (เหมือนเดิม) ---
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use.')
        
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load.')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true.')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack.')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RVIZ config file to use.')

    # =================================================================================
    # +++ START: ส่วนที่เพิ่มเข้ามา +++
    # =================================================================================

    # --- 1. รันระบบ EKF Localization ของเรา ---
    #    นี่คือการเรียกใช้ localization.launch.py ที่เราทำเสร็จแล้วสำหรับ Micro-ROS
    # start_ekf_localization_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         ws_dir, 'launch', 'localization.launch.py'
    #     )])
    # )

    # =================================================================================
    # +++ END: ส่วนที่เพิ่มเข้ามา +++
    # =================================================================================

    # --- Include Nav2 bringup (เหมือนเดิม) ---
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items(),
    )

    # --- Include RViz (เหมือนเดิม) ---
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'rviz_config': rviz_config,
            'use_sim_time': use_sim_time
        }.items()
    )

    # --- Assemble launch description (แก้ไขเล็กน้อย) ---
    ld = LaunchDescription()
    
    # เพิ่ม Arguments ก่อน (เหมือนเดิม)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_cmd)
    
    # เพิ่ม Action ที่จะรัน
    # ld.add_action(start_ekf_localization_cmd) # <--- !! เพิ่ม EKF ของเราเข้าไป !!
    ld.add_action(bringup_cmd)                # <--- Nav2 (เหมือนเดิม)
    ld.add_action(rviz_cmd)                   # <--- RViz (เหมือนเดิม)
    
    return ld