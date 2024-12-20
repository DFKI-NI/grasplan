#!/usr/bin/env python3

# Copyright (c) 2024 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

arm = moveit_commander.MoveGroupCommander('arm', wait_for_servers=10.0)


def test_go_to_cartesian_pose():
    '''
    goto cartesian pose with moveit
    '''

    # home pose expressed in cartesian coordinates, obtained via:
    # rosrun tf tf_echo world hand_ee_link
    target_pose = PoseStamped()

    # object in cartesian coordinates (is also recorded as "object" posture in srdf)
    translation = [-0.000950, 0.682979, 1.158076]
    rotation = [-0.116526, 0.609924, -0.207705, -0.755825]

    # home posture in cartesian coordinates
    # translation = [-0.288703, 0.154675, 1.592202]
    # rotation = [0.612385, -0.684744, -0.190438, -0.346182]

    # current pose, when doing bringup arm is already there
    # translation = [-0.596009, -0.291193, 1.481584]
    # rotation = [-0.703003, 0.082303, 0.701726, 0.081197]

    rospy.loginfo(f'planning frame : {arm.get_planning_frame()}')
    target_pose.header.frame_id = arm.get_planning_frame()  # 'world'
    target_pose.pose.position.x = translation[0]
    target_pose.pose.position.y = translation[1]
    target_pose.pose.position.z = translation[2]
    target_pose.pose.orientation.x = rotation[0]
    target_pose.pose.orientation.y = rotation[1]
    target_pose.pose.orientation.z = rotation[2]
    target_pose.pose.orientation.w = rotation[3]
    rospy.loginfo('going to cartesian pose')
    arm.set_pose_target(target_pose, end_effector_link='hand_ee_link')
    arm.go()
    rospy.loginfo('finished going to cartesian pose...')
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('goto_cart_test_node', anonymous=False)
    test_go_to_cartesian_pose()
