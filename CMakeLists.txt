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
    InsertObject.action
    PickObject.action
    PlaceObject.action
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
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    std_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/insert_obj_test_action_client
  scripts/object_recognition_mockup.py
  scripts/pick_obj_test_action_client
  scripts/place_obj_test_action_client
  scripts/publish_tf_world_to_robot
  scripts/rqt_grasplan
  scripts/rqt_planning_scene
  scripts/set_rviz_logger_level.py
  scripts/visualize_planning_scene_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
        plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
