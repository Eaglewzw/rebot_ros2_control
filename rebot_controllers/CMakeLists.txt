cmake_minimum_required(VERSION 3.8)
project(rebot_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  lifecycle_msgs
  std_msgs
  trajectory_msgs
  rebot_msgs
  kdl_parser
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ---------------------------------------------------------------------------
# Parameter libraries (generate_parameter_library)
# ---------------------------------------------------------------------------
generate_parameter_library(mit_joint_controller_parameters
  src/mit_joint_controller_parameters.yaml)
generate_parameter_library(gravity_compensation_controller_parameters
  src/gravity_compensation_controller_parameters.yaml)
generate_parameter_library(mit_trajectory_controller_parameters
  src/mit_trajectory_controller_parameters.yaml)
generate_parameter_library(joint_impedance_controller_parameters
  src/joint_impedance_controller_parameters.yaml)
generate_parameter_library(teleop_stream_controller_parameters
  src/teleop_stream_controller_parameters.yaml)

# ---------------------------------------------------------------------------
# Controller library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/kdl_gravity.cpp
  src/mit_joint_controller.cpp
  src/gravity_compensation_controller.cpp
  src/mit_trajectory_controller.cpp
  src/joint_impedance_controller.cpp
  src/teleop_stream_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  mit_joint_controller_parameters
  gravity_compensation_controller_parameters
  mit_trajectory_controller_parameters
  joint_impedance_controller_parameters
  teleop_stream_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface rebot_controllers.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}
    mit_joint_controller_parameters
    gravity_compensation_controller_parameters
    mit_trajectory_controller_parameters
    joint_impedance_controller_parameters
    teleop_stream_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_include_directories(test_rate_limiter PRIVATE include)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_include_directories(test_trajectory PRIVATE include)

  ament_add_gtest(test_kdl_gravity test/test_kdl_gravity.cpp)
  target_link_libraries(test_kdl_gravity ${PROJECT_NAME})

  ament_add_gtest(test_load_controllers test/test_load_controllers.cpp TIMEOUT 120)
  target_link_libraries(test_load_controllers ${PROJECT_NAME})
  ament_target_dependencies(test_load_controllers controller_manager)
endif()

ament_package()
