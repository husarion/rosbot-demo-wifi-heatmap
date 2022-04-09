cmake_minimum_required(VERSION 3.5)
project(nav2_read_rssi_at_waypoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbot_interfaces REQUIRED)
find_package(nav2_common REQUIRED)


nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_util
  nav2_core
  pluginlib
  rosbot_interfaces
)

add_library(${library_name} SHARED
  src/read_rssi_at_waypoint.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_waypoint_follower nav2_read_rssi_at_waypoint.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav2_read_rssi_at_waypoint.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()