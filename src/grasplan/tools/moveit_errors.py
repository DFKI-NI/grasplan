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

import threading
import time

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


# what a MoveIt error code means, in words the planner agent can repeat; codes not listed get their name only
MOVEIT_CODE_CAUSES = {
    'PLANNING_FAILED': 'no motion plan was found',
    'INVALID_MOTION_PLAN': 'the motion plan was invalid',
    'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE': 'the planning scene changed while the motion ran',
    'CONTROL_FAILED': 'the arm controller aborted the motion (a stop, a collision, or the arm left its path tolerance)',
    'UNABLE_TO_AQUIRE_SENSOR_DATA': 'no sensor data was available',
    'TIMED_OUT': 'the motion did not finish in time',
    'PREEMPTED': 'the motion was stopped before its end (cancelled or stopped)',
    'START_STATE_IN_COLLISION': 'the arm starts in collision',
    'GOAL_IN_COLLISION': 'the goal pose is in collision',
    'NO_IK_SOLUTION': 'the arm cannot reach the pose (no IK solution)',
    'FAILURE': 'a step of the motion failed',
}

# ur_dashboard_msgs/SafetyMode values that stop the arm, in words
_STOPPING_SAFETY_MODES = {
    'PROTECTIVE_STOP': 'protective stop',
    'SAFEGUARD_STOP': 'safeguard stop',
    'SYSTEM_EMERGENCY_STOP': 'emergency stop',
    'ROBOT_EMERGENCY_STOP': 'emergency stop',
    'VIOLATION': 'safety violation stop',
    'FAULT': 'safety fault stop',
    'AUTOMATIC_MODE_SAFEGUARD_STOP': 'safeguard stop',
    'SYSTEM_THREE_POSITION_ENABLING_STOP': 'enabling-device stop',
}


def moveit_code_name(code):
    '''the MoveItErrorCodes constant name of code, e.g. "CONTROL_FAILED"; "code <n>" when unknown'''
    for name, value in vars(MoveItErrorCodes).items():
        if name.isupper() and value == code:
            return name
    return f'code {code}'


def moveit_code_text(code):
    '''"CONTROL_FAILED (-4): the arm controller aborted the motion (...)" for a failure string'''
    name = moveit_code_name(code)
    cause = MOVEIT_CODE_CAUSES.get(name)
    label = f'{name} ({code})' if not name.startswith('code ') else f'MoveIt error code {code}'
    return f'{label}: {cause}' if cause else label


def motion_failure_text(what, code, tried=''):
    '''"the pick failed: CONTROL_FAILED (-4): ...; the arm is in a protective stop" for the MoveIt action paths
    that used to only print the code (the stop is asked after the failure, see arm_stop_text); a missing result (None,
    False: the action server did not answer or the goal was preempted) is worded as that. tried, e.g. "12 grasps of
    relay_1 tried, 4.1 s", says what was planned for, so a bare PLANNING_FAILED tells how hard the robot tried'''
    if code is None:
        return f'{what} failed: the MoveIt action server gave no result (not found within 2 s)'
    if code is False:
        return f'{what} was cancelled'
    detail = f' ({tried})' if tried else ''
    return f'{what} failed: {moveit_code_text(code)}{detail}{arm_stop_text()}'


def tried_text(count, noun, target, seconds, attempts=1):
    '''"12 grasps of relay_1 tried, 4.1 s" / "80 place poses on table_3 tried in 3 attempts, 47.4 s"; '' when
    nothing was counted'''
    if not count:
        return ''
    plural = noun if count == 1 else f'{noun}s'
    rounds = f' in {attempts} attempts' if attempts > 1 else ''
    return f'{count} {plural} {target} tried{rounds}, {seconds:.1f} s'


# service name -> the thread of a call whose reply never came (left behind by _reply_by)
_waiting_calls = {}
_waiting_lock = threading.Lock()


def _reply_by(service, service_type, deadline, what, discovery=0.5):
    '''
    The reply of an argument-free call of service, or None (and a log) when the service is not found within discovery
    seconds or gives no reply before deadline (a time.monotonic() value). Discovery and the call run in a daemon thread,
    so a service that takes the request but never replies cannot hold back the failed motion's result. Such a thread is
    left behind, at most one per service: later checks skip that service until it has replied, and its late reply
    reaches no one.
    '''
    with _waiting_lock:
        waiting = _waiting_calls.get(service)
        if waiting is not None and waiting.is_alive():
            rospy.logwarn(f'{what} not checked ({service}): an earlier call still waits for its reply')
            return None
        _waiting_calls.pop(service, None)
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        rospy.logwarn(f'{what} not checked ({service}): no time left for it')
        return None
    reply = []

    def call():
        try:
            rospy.wait_for_service(service, min(discovery, remaining))
            reply.append(rospy.ServiceProxy(service, service_type)())
        except Exception as error:  # any failure of the lookup only drops the extra words
            reply.append(error)

    worker = threading.Thread(target=call, name=f'reply_by {service}', daemon=True)
    worker.start()
    worker.join(max(deadline - time.monotonic(), 0.0))
    if worker.is_alive():
        with _waiting_lock:
            _waiting_calls[service] = worker
        rospy.logwarn(f'{what} not checked ({service}): no reply within {remaining:.1f} s')
        return None
    if isinstance(reply[0], Exception):
        rospy.logwarn(f'{what} not checked ({service}): {reply[0]}')
        return None
    return reply[0]


def arm_stop_text(service='ur_hardware_interface/dashboard/get_safety_mode', timeout=1.0):
    '''
    "; the arm is in a protective stop (UR safety mode PROTECTIVE_STOP)" while the UR safety mode stops the arm,
    else the arm controller's state (arm_controller_text) or ''. Asked after a failed motion so the planner hears about
    a stop (the sim cable monitor presses one) in the failure itself and not only from a later event. Never raises and
    returns within timeout seconds for the whole check: a missing or silent dashboard or controller manager gives ''
    and a log.
    '''
    deadline = time.monotonic() + timeout
    try:
        from ur_dashboard_msgs.msg import SafetyMode
        from ur_dashboard_msgs.srv import GetSafetyMode
    except ImportError as error:
        rospy.logwarn(f'arm safety mode not checked: {error}')
        return arm_controller_text(deadline=deadline)
    reply = _reply_by(service, GetSafetyMode, deadline, 'arm safety mode')
    if reply is None:
        return arm_controller_text(deadline=deadline)
    for name, words in _STOPPING_SAFETY_MODES.items():
        if getattr(SafetyMode, name, None) == reply.safety_mode.mode:
            article = 'an' if words[0] in 'aeiou' else 'a'
            return f'; the arm is in {article} {words} (UR safety mode {name})'
    return arm_controller_text(deadline=deadline)


def arm_controller_text(service='controller_manager/list_controllers', name='arm_controller', deadline=None):
    '''
    "; the arm controller arm_controller is not running (stopped): ..." when the controller that runs the arm
    trajectories (name) is loaded but not running, else ''. A stopped joint
    trajectory controller rejects every goal (INVALID_GOAL, no text), e.g. while an e-stop is pressed or after a stop
    was released again before the safety mode was asked. Never raises and returns by deadline (time.monotonic(),
    default half a second from now): no controller manager or no reply gives '' and a log.
    '''
    deadline = time.monotonic() + 0.5 if deadline is None else deadline
    try:
        from controller_manager_msgs.srv import ListControllers
    except ImportError as error:
        rospy.logwarn(f'arm controller state not checked: {error}')
        return ''
    reply = _reply_by(service, ListControllers, deadline, 'arm controller state')
    for controller in reply.controller if reply is not None else ():
        if controller.name == name and controller.state != 'running':
            return (f'; the arm controller {name} is not running ({controller.state}): an e-stop, a protective stop or '
                    f'a controller switch stopped it, so it rejects arm motions until it runs again')
    return ''
