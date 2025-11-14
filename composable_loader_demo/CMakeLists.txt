cmake_minimum_required(VERSION 3.8)
project(composable_loader_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- SERVICE-GENERIERUNG ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadComposableNode.srv"
  DEPENDENCIES rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# --- SERVER ---
add_executable(load_composable_node_server src/load_composable_node_server.cpp)
ament_target_dependencies(load_composable_node_server
  rclcpp rclcpp_components rcl_interfaces)

# WICHTIG: Damit der Service-Header verfügbar ist!
rosidl_target_interfaces(load_composable_node_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- CLIENT ---
add_executable(load_composable_node_client src/load_composable_node_client.cpp)
ament_target_dependencies(load_composable_node_client
  rclcpp rcl_interfaces)

rosidl_target_interfaces(load_composable_node_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  load_composable_node_server
  load_composable_node_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})

ament_package()
