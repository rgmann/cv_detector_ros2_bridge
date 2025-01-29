cmake_minimum_required(VERSION 3.8)
project(cv_detector_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(FLATBUFFERS_SRC_DIR ./dep/flatbuffers-25.1.24)
add_subdirectory(${FLATBUFFERS_SRC_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/flatbuffers-build
                 EXCLUDE_FROM_ALL)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Rect.msg"
  "msg/Detection.msg"
  "msg/Meta.msg"
  "msg/DetectionsList.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_custom_command(
  OUTPUT src/detections_list_generated.h
  COMMAND flatc --cpp -o ${CMAKE_CURRENT_SOURCE_DIR}/src/ ${CMAKE_CURRENT_SOURCE_DIR}/src/fb_schema/detections_list.fbs
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/fb_schema/detections_list.fbs
)

add_executable(detections_ros2_bridge
  src/convert.cpp
  src/convert.hpp
  src/detections_ros2_bridge.cpp
  src/detections_list_generated.h
  src/detections_list_client.cpp)
ament_target_dependencies(detections_ros2_bridge rclcpp geometry_msgs)
target_link_libraries(detections_ros2_bridge flatbuffers)
target_link_libraries(detections_ros2_bridge "${cpp_typesupport_target}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
