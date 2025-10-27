cmake_minimum_required(VERSION 3.8)
project(warehouse_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(include)

add_library(warehouse_manager_lib SHARED
	src/warehouse_manager.cpp
	src/node_interfaces.cpp
  src/pose_logger.cpp
)
ament_target_dependencies(warehouse_manager_lib 
  rclcpp 
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  )

add_executable(warehouse_manager_node src/warehouse_manager_node.cpp)
target_link_libraries(warehouse_manager_node warehouse_manager_lib)
ament_target_dependencies(warehouse_manager_node 
  rclcpp 
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros)

install(TARGETS
  warehouse_manager_lib
  warehouse_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
