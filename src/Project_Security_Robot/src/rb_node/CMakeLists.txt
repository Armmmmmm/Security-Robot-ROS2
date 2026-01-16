cmake_minimum_required(VERSION 3.5)
project(rb_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
add_executable(joint_nodeRB src/joint_nodeRB.cpp)
add_executable(odometryRB src/odometryRB.cpp)


ament_target_dependencies(joint_nodeRB rclcpp)
ament_target_dependencies(joint_nodeRB rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs tf2_ros)
ament_target_dependencies(odometryRB rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs tf2_ros nav_msgs)

# ament_target_dependencies(joint_state_node rclcpp sensor_msgs std_msgs)


install(TARGETS
  joint_nodeRB 
  odometryRB
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
