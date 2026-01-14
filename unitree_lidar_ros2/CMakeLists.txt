cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0074 NEW)

project(unitree_lidar_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# SDK include path
include_directories(
  include
  /usr/local/include/unitree_lidar_sdk
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# SDK library path by architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_LIDAR_SDK_LIB
    /usr/local/lib/unitree_lidar_sdk/aarch64/libunilidar_sdk2.a
  )
else()
  set(UNITREE_LIDAR_SDK_LIB
    /usr/local/lib/unitree_lidar_sdk/x86_64/libunilidar_sdk2.a
  )
endif()

add_executable(unitree_lidar_ros2_node
  src/unitree_lidar_ros2_node.cpp
)

target_link_libraries(
  unitree_lidar_ros2_node
  ${UNITREE_LIDAR_SDK_LIB}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  unitree_lidar_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

install(TARGETS
  unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

ament_package()
