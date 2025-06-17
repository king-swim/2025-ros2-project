cmake_minimum_required(VERSION 3.5)
project(embedded_project)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

############################################################################
# TODO: Define srv/SetFilterConfig.srv
# - Use rosidl_generate_interfaces, ament_export_dependencies
# - Dependencies : sensor_msgs, geometry_msgs, std_msgs, builtin_interfaces
############################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFilterConfig.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(
  sensor_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
)

add_library(sensor_utils
  src/sensor_utils.cpp
)

ament_target_dependencies(sensor_utils
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  OpenCV
)

############################################################################
# TODO: Define an executable named service_node using src/service_node.cpp
# - Use add_executable and ament_target_dependencies
# - Dependencies: rclcpp, sensor_msgs, geometry_msgs, OpenCV
############################################################################
add_executable(service_node src/service_node.cpp)

ament_target_dependencies(service_node
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
)

rosidl_target_interfaces(service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(service_node sensor_utils)

############################################################################
# TODO: Define an executable named client_node using src/client_node.cpp
# - Use add_executable and ament_target_dependencies
# - Dependencies: rclcpp, sensor_msgs, geometry_msgs
############################################################################
add_executable(client_node src/client_node.cpp)

ament_target_dependencies(client_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

rosidl_target_interfaces(client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

############################################################################
# TODO: Install the service node and client node
############################################################################
install(TARGETS
  client_node
  service_node
  sensor_utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
