from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to start the imu_filter_madgwick node.
    """
    
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'use_mag': False},      # MPU6050 ไม่มี Magnetometer
            {'publish_tf': False},   # EKF จะเป็นตัว publish TF
        ],
        remappings=[
            # รับข้อมูลดิบจาก /imu/data_raw (ซึ่งเป็นค่าเริ่มต้น ไม่ต้องแก้)
            # ส่งข้อมูลที่กรองแล้วออกไปที่ /imu/data_filtered
            ('/imu/data', '/imu/data_filtered')
        ]
    )

    return LaunchDescription([
        imu_filter_node
    ])