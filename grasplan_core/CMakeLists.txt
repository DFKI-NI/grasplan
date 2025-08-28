cmake_minimum_required(VERSION 3.22)
project(grasplan_core)

# compile with C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

install(
  DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  ${PROJECT_NAME}/pick.py
  ${PROJECT_NAME}/mtc_pick.py
  ${PROJECT_NAME}/place.py
  ${PROJECT_NAME}/insert.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
