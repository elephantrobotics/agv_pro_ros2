cmake_minimum_required(VERSION 3.8)
project(agv_pro_multi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  tf2
  tf2_ros
)

add_library(agv_pro_multi_layers SHARED src/robot_footprint_layer.cpp)

target_include_directories(agv_pro_multi_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(agv_pro_multi_layers ${dependencies})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS ${PROJECT_NAME}/agv_pro_multi_teleop.py
  DESTINATION lib/${PROJECT_NAME} RENAME agv_pro_multi_teleop)
install(PROGRAMS ${PROJECT_NAME}/pub_follower_goal.py
  DESTINATION lib/${PROJECT_NAME} RENAME pub_follower_goal)
install(PROGRAMS ${PROJECT_NAME}/get_follower_point.py
  DESTINATION lib/${PROJECT_NAME} RENAME get_follower_point)

install(TARGETS agv_pro_multi_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(agv_pro_multi_layers)
ament_export_dependencies(${dependencies})

ament_package()
