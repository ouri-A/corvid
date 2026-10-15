cmake_minimum_required(VERSION 3.8)
project(corvid_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Our messages embed types from these, so they must be found before the
# generator runs.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generates Python and C++ types from each .msg listed. Every new message
# file has to be added here or it is silently not built.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetection.msg"
  "msg/MarkerDetectionArray.msg"
  "msg/StationKeeperStatus.msg"
  "msg/CoordinatorStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Passes the runtime dependency on to packages that depend on this one.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
