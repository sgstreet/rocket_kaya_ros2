cmake_minimum_required(VERSION 3.5)
project(kaya_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(kaya_base SHARED
	src/kaya_base.cpp
	src/kaya_joints.cpp
	src/group_bulk_read.cpp
	src/port_handler.cpp
	src/protocol2_packet_handler.cpp
	src/group_bulk_write.cpp
	src/packet_handler.cpp
	src/port_handler_linux.cpp
	src/group_sync_write.cpp
	src/group_sync_read.cpp)

rclcpp_components_register_nodes(kaya_base "kaya_base::KayaBase")

#target_compile_definitions(composition_nodes PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(kaya_base rclcpp rclcpp_components std_msgs rclcpp_lifecycle lifecycle_msgs sensor_msgs geometry_msgs)

add_executable(kaya_base_node src/kaya_base_node.cpp)
target_link_libraries(kaya_base_node kaya_base ${rclcpp_lifecycle_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  kaya_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  kaya_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
