cmake_minimum_required(VERSION 3.8)
project(datmo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(msg_files
  "msg/Track.msg"
  "msg/TrackArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  nav_msgs
 )

ament_export_dependencies(rosidl_default_runtime)

include_directories(include/)
# add_executable(kalman src/kalman.cpp)
# ament_target_dependencies(kalman Eigen3)

# add_executable(l_shape_tracker src/l_shape_tracker.cpp src/kalman.cpp)

add_executable(datmo_node src/main.cpp src/datmo_node.cpp src/cluster.cpp src/l_shape_tracker.cpp src/kalman.cpp)
ament_target_dependencies(datmo_node rclcpp visualization_msgs tf2 tf2_geometry_msgs )
# ament_target_dependencies(cluster rclcpp visualization_msgs tf2 tf2_geometry_msgs ${PROJECT_NAME})

install(TARGETS
  datmo_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

rosidl_target_interfaces(datmo_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
