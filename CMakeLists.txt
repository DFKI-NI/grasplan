cmake_minimum_required(VERSION 3.0.2)
project(grasplan)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  message_generation
  rqt_gui
  rqt_gui_py
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PickObject.action
  PlaceObject.action
  InsertObject.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  config/object_recognition_mockup/objBounds.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    std_msgs
)
