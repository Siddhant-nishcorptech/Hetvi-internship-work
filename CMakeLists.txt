cmake_minimum_required(VERSION 3.8)
project(my_bot_urdf_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node
    rclcpp
    geometry_msgs
)

add_executable(stair_detector_node src/stair_detector_node.cpp)

target_include_directories(stair_detector_node PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stair_detector_node
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

ament_target_dependencies(stair_detector_node
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    pcl_conversions
    visualization_msgs
)

add_executable(voxel_filter_node src/voxel_filter_node.cpp)

target_include_directories(voxel_filter_node PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(voxel_filter_node
    ${PCL_LIBRARIES}
)

ament_target_dependencies(voxel_filter_node
    rclcpp
    sensor_msgs
    std_msgs
    pcl_conversions
)

add_executable(controller_node src/controller.cpp)

ament_target_dependencies(controller_node
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  ros_gz_interfaces
)

install(DIRECTORY
  urdf launch config meshes rviz worlds 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  teleop_node
  controller_node
  stair_detector_node
  voxel_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()