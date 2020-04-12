cmake_minimum_required(VERSION 3.5)
project(kaya_sound_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
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
find_package(kaya_sound_interface REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GST gstreamer-1.0 REQUIRED)
pkg_check_modules(GST_APP gstreamer-app-1.0 REQUIRED)

include_directories(include 
${INCLUDE_DIRS}
${GST_INCLUDE_DIRS}
${GST_APP_INCLUDE_DIRS}
)

add_library(kaya_sound_core SHARED src/kaya_sound_core.cpp)
target_link_libraries(kaya_sound_core ${GST_LIBRARIES} ${GST_APP_LIBRARIES}) 
ament_target_dependencies(kaya_sound_core rclcpp rclcpp_components std_msgs kaya_sound_interface)

rclcpp_components_register_nodes(kaya_sound_core "kaya_sound::KayaSoundCore")

add_executable(kaya_sound_core_node src/kaya_sound_core_node.cpp)
ament_target_dependencies(kaya_sound_core_node rclcpp)
target_link_libraries(kaya_sound_core_node kaya_sound_core)

install(
  TARGETS kaya_sound_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS kaya_sound_core_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

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
