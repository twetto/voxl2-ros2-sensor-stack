cmake_minimum_required(VERSION 3.8)
project(voxl_h265_decoder)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(h265_decoder_node src/h265_decoder_node.cpp)
target_compile_features(h265_decoder_node PUBLIC cxx_std_17)
ament_target_dependencies(
  h265_decoder_node
  rclcpp
  sensor_msgs
)
target_link_libraries(h265_decoder_node PkgConfig::GSTREAMER)

install(TARGETS h265_decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
