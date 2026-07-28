cmake_minimum_required(VERSION 3.8)
project(rebot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, and RViz files
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Test setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
