cmake_minimum_required(VERSION 3.0.2)
project(grasplan)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/object_recognition_mockup/objBounds.cfg
)

catkin_package()
