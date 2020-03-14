cmake_minimum_required(VERSION 3.10)

project(kaya_realsense)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -W-pendantic)
endif()

find_package(ament_cmake REQUIRED)

set(PACKAGE_VERSION "2.33.1")

set(extra_cmake_args)
  
if(DEFINED CMAKE_BUILD_TYPE)
  list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
endif()

list(APPEND extra_cmake_args "-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc")
list(APPEND extra_cmake_args "-DBUILD_EXAMPLES=false")
list(APPEND extra_cmake_args "-DBUILD_GRAPHICAL_EXAMPLES=false")
list(APPEND extra_cmake_args "-DFORCE_RSUSB_BACKEND=true")
list(APPEND extra_cmake_args "-DHWM_OVER_XU=true")
list(APPEND extra_cmake_args "-DBUILD_WITH_TM2=true")
list(APPEND extra_cmake_args "-DBUILD_WITH_CUDA=true")
list(APPEND extra_cmake_args "-DBUILD_GLSL_EXTENSIONS=false")
list(APPEND extra_cmake_args "-DIMPORT_DEPTH_CAM_FW=true")
list(APPEND extra_cmake_args "-DNO_LDCONFIG=true")

if(DEFINED CMAKE_TOOLCHAIN_FILE)
  list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
else()
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
endif()

include(ExternalProject)
  
ExternalProject_Add(realsense
  GIT_REPOSITORY https://github.com/sgstreet/librealsense.git
  GIT_TAG kaya_realsense
  GIT_SHALLOW TRUE
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install ${extra_cmake_args}
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}
)

ament_export_include_directories(include)
ament_export_libraries(librealsense2.so)

ament_package()
