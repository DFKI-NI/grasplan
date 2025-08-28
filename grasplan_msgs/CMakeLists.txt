cmake_minimum_required(VERSION 3.22)
project(grasplan_msgs)

# compile with C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Register your action files
rosidl_generate_interfaces(${PROJECT_NAME}
  action/PickObject.action
  action/PlaceObject.action
  action/InsertObject.action
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
