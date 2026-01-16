cmake_minimum_required(VERSION 3.5)
project(mobile_robotics_update)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# เพิ่ม executable ทั้งหมด
add_executable(odometryRB src/odometryRB.cpp)
add_executable(joint_nodeRB src/joint_nodeRB.cpp)
add_executable(imu_tf_broadcaster_node src/imu_tf_broadcaster_node.cpp)

# ตั้งค่า dependencies สำหรับแต่ละ executable
ament_target_dependencies(
  odometryRB
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  joint_nodeRB
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(
  imu_tf_broadcaster_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS 
  joint_nodeRB
  odometryRB
  imu_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

# ติดตั้ง Directory อื่นๆ (บรรทัดนี้ถูกต้องแล้ว เพราะมี config อยู่)
install(
  DIRECTORY launch urdf meshes config map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()