cmake_minimum_required(VERSION 3.8)
project(rebot_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Test setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
