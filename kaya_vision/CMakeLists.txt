cmake_minimum_required(VERSION 3.5)
project(kaya_vision)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(resource_retriever REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(kaya_realsense REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(kaya_vision SHARED src/kaya_vision_manager.cpp src/kaya_vision_chatter.cpp src/kaya_realsense_camera.cpp src/kaya_tf_extrinsics.cpp)
ament_target_dependencies(kaya_vision rclcpp rclcpp_components rclcpp_lifecycle lifecycle_msgs tf2_ros sensor_msgs geometry_msgs cv_bridge OpenCV resource_retriever kaya_realsense)
target_link_libraries(kaya_vision resource_retriever::resource_retriever)

rclcpp_components_register_nodes(${PROJECT_NAME} "kaya_vision::KayaVisionManager" "kaya_vision::KayaVisionChatter" "kaya_vision::KayaRealsenseCamera")

add_executable(kaya_vision_node src/kaya_vision_node.cpp)
ament_target_dependencies(kaya_vision_node rclcpp rclcpp_components rclcpp_lifecycle lifecycle_msgs tf2_ros sensor_msgs geometry_msgs cv_bridge OpenCV resource_retriever kaya_realsense)
target_link_libraries(kaya_vision_node kaya_vision)

install(TARGETS
  kaya_vision_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  kaya_vision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
