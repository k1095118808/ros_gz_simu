cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_bringup)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_example_description REQUIRED)
find_package(ros_gz_example_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/pose_to_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

