cmake_minimum_required(VERSION 3.8)
project(rebot_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install MoveIt configuration files
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Test setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
