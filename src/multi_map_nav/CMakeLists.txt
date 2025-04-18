cmake_minimum_required(VERSION 3.8)
project(multi_map_nav)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2_ros REQUIRED)

# === ACTION FILES ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiMapNavigate.action"
  DEPENDENCIES action_msgs geometry_msgs
)

# === INCLUDE PATH ===
include_directories(
  include
)

# === WORMHOLE DB LIBRARY ===
add_library(wormhole_db SHARED
  src/wormhole_db.cpp
)

target_link_libraries(wormhole_db
  sqlite3
)

ament_target_dependencies(wormhole_db
  rclcpp
  geometry_msgs
)

# === NAVIGATION SERVER EXECUTABLE ===
add_executable(navigation_server
  src/navigation_server.cpp
)

target_link_libraries(navigation_server
  wormhole_db
  sqlite3
)

ament_target_dependencies(navigation_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# === MAP VISUALIZER EXECUTABLE ===
add_executable(map_visualizer
  src/map_visualizer.cpp
)

target_link_libraries(map_visualizer
  wormhole_db
  sqlite3
)

ament_target_dependencies(map_visualizer
  rclcpp
  geometry_msgs
  visualization_msgs
)

# === MAP SWITCHER EXECUTABLE ===
add_executable(map_switcher
  src/map_switcher_node.cpp
)

ament_target_dependencies(map_switcher
  rclcpp
  std_msgs
  nav_msgs
)

# === ROSIDL INTERFACES FOR EXECUTABLE ===
# Using the recommended approach instead of rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_server "${cpp_typesupport_target}")

# === VDA5050 CLIENT EXECUTABLE ===
add_executable(vda5050_client
  src/vda5050_client.cpp
)

ament_target_dependencies(vda5050_client
  rclcpp
  std_msgs
  nlohmann_json
)

# === NAVIGATION CLIENT EXECUTABLE ===
add_executable(navigation_client
  src/navigation_client.cpp
)

ament_target_dependencies(navigation_client
  rclcpp
  rclcpp_action
  geometry_msgs
)

target_link_libraries(navigation_client "${cpp_typesupport_target}")

# === TF PUBLISHER NODE EXECUTABLE ===
add_executable(tf_publisher_node
  src/tf_publisher_node.cpp
)

ament_target_dependencies(tf_publisher_node
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
  std_msgs
)

# === ROBOT STATE PUBLISHER NODE EXECUTABLE ===
add_executable(robot_state_publisher_node
  src/robot_state_publisher_node.cpp
)

ament_target_dependencies(robot_state_publisher_node
  rclcpp
  std_msgs
)

# === INSTALLS ===
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries to the lib directory
install(TARGETS
  wormhole_db
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  navigation_server
  map_visualizer
  map_switcher
  vda5050_client
  navigation_client
  tf_publisher_node
  robot_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Explicitly disable ament_python part to avoid the symlink conflict
if(FALSE)
  # Install Python modules
  ament_python_install_package(${PROJECT_NAME})
endif()

# Install Python scripts directly
install(
  PROGRAMS
  scripts/setup_wormholes.py
  scripts/verify_map_publishing.py
  scripts/generate_maps.py
  scripts/direct_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)

# === FINAL PACKAGE ===
ament_package()
