# grasplan

Simple grasp planning for robots.

This package is under active delopment, so things might not work perfectly well.

It has been tested under ubuntu 20.04 and ros noetic mainly with the mobipick robot (mir base + ur5 arm).

# Usage

To teach grasp poses run:

    roslaunch mobipick_pick_n_place teach_grasp_poses.launch object_name:=multimeter

To edit grasp poses do:

    roslaunch grasplan grasp_editor.launch

Unfortunately I don't have much time right now to document properly the package, this will have to wait. Thank you for your patience though!
