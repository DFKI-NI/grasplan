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
from moveit_msgs.msg import MoveItErrorCodes


def print_moveit_error_helper(error_code, moveit_error_code, moveit_error_string):
    '''
    function that helps print_moveit_error method to have less code
    '''
    if error_code == moveit_error_code:
        rospy.logwarn(f'moveit says : {moveit_error_string}')


def print_moveit_error(error_code):
    '''
    receive moveit result, compare with error codes, print what happened
    '''
    print_moveit_error_helper(error_code, MoveItErrorCodes.PLANNING_FAILED, 'PLANNING_FAILED')
    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_MOTION_PLAN, 'INVALID_MOTION_PLAN')
    print_moveit_error_helper(
        error_code,
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
        'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    )
    print_moveit_error_helper(error_code, MoveItErrorCodes.CONTROL_FAILED, 'CONTROL_FAILED')
    print_moveit_error_helper(error_code, MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA, 'UNABLE_TO_AQUIRE_SENSOR_DATA')
    print_moveit_error_helper(error_code, MoveItErrorCodes.TIMED_OUT, 'TIMED_OUT')
    print_moveit_error_helper(error_code, MoveItErrorCodes.PREEMPTED, 'PREEMPTED')

    print_moveit_error_helper(error_code, MoveItErrorCodes.START_STATE_IN_COLLISION, 'START_STATE_IN_COLLISION')
    print_moveit_error_helper(
        error_code, MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS, 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    )

    print_moveit_error_helper(error_code, MoveItErrorCodes.GOAL_IN_COLLISION, 'GOAL_IN_COLLISION')
    print_moveit_error_helper(
        error_code, MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS, 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    )
    print_moveit_error_helper(error_code, MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED, 'GOAL_CONSTRAINTS_VIOLATED')

    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_GROUP_NAME, 'INVALID_GROUP_NAME')
    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS, 'INVALID_GOAL_CONSTRAINTS')
    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_ROBOT_STATE, 'INVALID_ROBOT_STATE')
    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_LINK_NAME, 'INVALID_LINK_NAME')
    print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_OBJECT_NAME, 'INVALID_OBJECT_NAME')

    print_moveit_error_helper(error_code, MoveItErrorCodes.FRAME_TRANSFORM_FAILURE, 'FRAME_TRANSFORM_FAILURE')
    print_moveit_error_helper(
        error_code, MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE, 'COLLISION_CHECKING_UNAVAILABLE'
    )
    print_moveit_error_helper(error_code, MoveItErrorCodes.ROBOT_STATE_STALE, 'ROBOT_STATE_STALE')
    print_moveit_error_helper(error_code, MoveItErrorCodes.SENSOR_INFO_STALE, 'SENSOR_INFO_STALE')
    print_moveit_error_helper(error_code, MoveItErrorCodes.COMMUNICATION_FAILURE, 'COMMUNICATION_FAILURE')

    print_moveit_error_helper(error_code, MoveItErrorCodes.NO_IK_SOLUTION, 'NO_IK_SOLUTION')
