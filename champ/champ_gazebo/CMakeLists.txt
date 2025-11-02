cmake_minimum_required(VERSION 3.10)
project(champ_gazebo)  # Must match package.xml

# Use C++17
add_compile_options(-std=c++17)

# --------------------------
# Find ROS 2 packages
# --------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)

# --------------------------
# Find Gazebo Harmonic packages
# --------------------------
find_package(gz-sim8 REQUIRED)
find_package(gz-physics7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz_ros2_control REQUIRED)

# --------------------------
# Include directories
# --------------------------
include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}         # <-- Add URDF includes
)

# --------------------------
# Dependencies list for ament
# --------------------------
set(dependencies
  rclcpp
  urdf
  champ
  champ_msgs
  gz_ros2_control
)

# --------------------------
# Build C++ executable
# --------------------------
add_executable(contact_sensor src/contact_sensor.cpp)

ament_target_dependencies(contact_sensor
  rclcpp
  champ
  champ_msgs
  urdf                       # <-- Link URDF dependency
)

# Link Gazebo libraries
target_link_libraries(contact_sensor
  gz-sim8::gz-sim8
  gz-physics7::gz-physics7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-sensors8::gz-sensors8
)

# --------------------------
# Install executable
# --------------------------
install(TARGETS contact_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------
# Install Python scripts
# --------------------------
install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION bin
)

# --------------------------
# Install launch/config/worlds directories
# --------------------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# --------------------------
# Export includes and dependencies
# --------------------------
ament_export_include_directories(include ${urdf_INCLUDE_DIRS})
ament_export_dependencies(${dependencies})

# --------------------------
# Finalize package
# -----------------------
ament_package()