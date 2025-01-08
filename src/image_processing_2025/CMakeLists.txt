cmake_minimum_required(VERSION 3.5)

project(image_processing_2025)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
include_directories(image_processing_2025)

# Install project launch files

install(
  DIRECTORY
    scripts image_processing_2025
  DESTINATION share/${PROJECT_NAME}
)


install(
  PROGRAMS scripts/camera_node.py scripts/yolopv2_demo.py scripts/detection_and_classification.py scripts/yolopX_demo.py scripts/only_detection_and_classification.py
  DESTINATION lib/${PROJECT_NAME}
)

set(msg_files
  "msg/DetectedSign.msg"
  "msg/SignList.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
