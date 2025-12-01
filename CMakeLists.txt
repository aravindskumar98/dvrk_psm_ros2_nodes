cmake_minimum_required(VERSION 3.8)
project(dvrk_psm_ros2_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crtk_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find Qt5 (required by cisst)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Xml XmlPatterns)

# Find cisst libraries
# cisst uses its own CMake configuration
find_package(cisst REQUIRED COMPONENTS
  cisstCommon
  cisstVector
  cisstNumerical
  cisstRobot
)

# Include cisst use file if found
if(cisst_FOUND)
  include(${CISST_USE_FILE})
endif()

# Include directories
include_directories(
  include
  ${cisst_INCLUDE_DIRS}
)

# ==============================================================================
# PSM Kinematic Node
# ==============================================================================
add_executable(psm_kinematic_ros2node
  src/psm_kinematic_ros2node.cpp
)

ament_target_dependencies(psm_kinematic_ros2node
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  crtk_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link cisst libraries
target_link_libraries(psm_kinematic_ros2node
  ${cisst_LIBRARIES}
  cisstRobot
)

# Apply cisst compile definitions
cisst_target_link_libraries(psm_kinematic_ros2node ${CISST_LIBRARIES})

# ==============================================================================
# PSM Joint Level Node
# ==============================================================================
add_executable(psm_jointlevel_ros2node
  src/psm_jointlevel_ros2node.cpp
)

ament_target_dependencies(psm_jointlevel_ros2node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  crtk_msgs
)

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS
  psm_kinematic_ros2node
  psm_jointlevel_ros2node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# ==============================================================================
# Testing
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
