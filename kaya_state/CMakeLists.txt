cmake_minimum_required(VERSION 3.5)
project(kaya_state)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME} SHARED
  src/kaya_state.cpp
  src/kaya_joint_state.cpp)
  
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  kdl_parser
  orocos_kdl
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  urdf
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "kaya_state::KayaState"
  "kaya_state::KayaJointState"
)

add_executable(kaya_state_node
  src/kaya_state_node.cpp
)

ament_target_dependencies(kaya_state_node
  kdl_parser
  orocos_kdl
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  urdf
)
target_link_libraries(kaya_state_node
  ${PROJECT_NAME}
)

add_executable(kaya_joint_state_node
  src/kaya_joint_state_node.cpp
 )

ament_target_dependencies(kaya_joint_state_node
  rclcpp
  sensor_msgs
  urdf
)

target_link_libraries(kaya_joint_state_node
  ${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS kaya_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS kaya_joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_dependencies(
  kdl_parser
  orocos_kdl
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  urdf)
  
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
