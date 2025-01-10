cmake_minimum_required(VERSION 3.5)
project(drone_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose3D.msg"  # Ensure this is the correct path to Pose3D.msg
  DEPENDENCIES std_msgs geometry_msgs  # Add any other dependencies
)

# Install the message files
ament_export_dependencies(rosidl_default_runtime)
ament_package()

