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

# action client helper
import actionlib
from actionlib_msgs.msg import GoalStatus

# trajectory canceller
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
import re  # regular expressions


class TrajectoryCanceller:
    """
    Helper class that:
    1) keeps track of goals, adds them to a set and removes them if completed.
    2) offers a cancel_all_goals method that iterates over unfinished tracked goals and sends cancellations
    via topic to each of them.

    Motivation for this helper class:
    Sometimes rogue action servers ignore cancellation requests.
    As a workaround once can send such cancellation requests directly to the joint trajectory controller,
    effectively crippling the ability to actuate the arm or the gripper.
    """

    def __init__(self, base_topic):
        """
        base_topic example: '/namespace/arm_controller/follow_joint_trajectory/'
        '_controller' str should be there, because is used to compute the canceller_type
        which is used for logging purposes...
        """
        # programatically get the name of the canceller
        match = re.search(r'/(\w+)_controller', base_topic)
        self.canceller_type = match.group(1) if match else 'unknown'

        goal_topic = base_topic + 'goal'
        cancel_topic = base_topic + 'cancel'
        status_topic = base_topic + 'status'

        self.cancel_pub = rospy.Publisher(cancel_topic, GoalID, queue_size=1)
        self.goal_set = set()  # Set to hold unique goal_ids

        rospy.Subscriber(goal_topic, FollowJointTrajectoryActionGoal, self.goal_callback)
        rospy.Subscriber(status_topic, GoalStatusArray, self.status_callback)
        rospy.loginfo(
            f"TrajectoryCanceller initialized for goal topic '{goal_topic}', "
            f"cancel topic '{cancel_topic}', and status topic '{status_topic}'"
        )

    def goal_callback(self, msg):
        # Add the new goal_id to the set if it's not already present
        goal_id = msg.goal_id.id
        if goal_id not in self.goal_set:
            self.goal_set.add(goal_id)
            rospy.loginfo(f'Captured new goal_id for {self.canceller_type}: {goal_id}')

    def status_callback(self, msg):
        # Iterate over each status in the status_list
        for status in msg.status_list:
            goal_id = status.goal_id.id
            goal_status = status.status
            # If the status is SUCCEEDED, remove it from the set
            if goal_status == GoalStatus.SUCCEEDED:
                if goal_id in self.goal_set:
                    self.goal_set.remove(goal_id)
                    rospy.loginfo(f'Goal {goal_id} succeeded and removed from set ({self.canceller_type})')

    def has_active_goals(self):
        # Check if there are any goals in the set
        return len(self.goal_set) > 0

    def cancel_all_goals(self):
        if self.has_active_goals():
            # Iterate over all goal_ids in the set, sending a cancel request for each
            while self.goal_set:
                goal_id = self.goal_set.pop()  # Remove any goal_id from the set
                cancel_msg = GoalID()
                cancel_msg.id = goal_id
                self.cancel_pub.publish(cancel_msg)
                rospy.loginfo(f'Sent {self.canceller_type} cancel request for goal_id: {goal_id}')
            return True
        else:
            rospy.logwarn(f'No active goals to cancel for {self.canceller_type}')
            return False


class ActionClientHelper:
    """
    A customized alternative to action_client.wait_for_result(...)
    that includes support for bypassing unresponsive action servers (e.g., MoveIt pickup server).
    If the server ignores preemption requests, this method forcibly cancels goals
    by directly interacting with the joint trajectory controllers.
    """

    def __init__(self, ns, action_server, controller_names):
        """
        Initializes the action server helper.

        :param ns: The namespace under which the action server and controllers are operating.
        :param type: A string indicating the type of action server, e.g., 'pick', 'place', or 'insert'.
        :param action_server: An instance of the actionlib server that manages the corresponding actions.
        :param controller_names: A list of controller names, e.g., ['arm', 'gripper'],
                                used to create trajectory cancellers for each controller.

        Example:
            If `ns` is '/robot_namespace' and `controller_names` is ['arm', 'gripper'],
            the trajectory canceller topics will be:
                - /robot_namespace/arm_controller/follow_joint_trajectory/cancel
                - /robot_namespace/gripper_controller/follow_joint_trajectory/cancel
        """
        self.canceller_list = []
        for controller_name in controller_names:
            self.canceller_list.append(
                TrajectoryCanceller(f'/{ns}/{controller_name}_controller/follow_joint_trajectory/')
            )

        self.action_server = action_server

    def send_goal_to_rogue_server_and_wait(self, goal, action_client, patience_timeout=0.1):
        """
        Executes an action with support for forced preemption when the action server is unresponsive.

        :param goal: The goal to send to the action server.
        :param action_client: The SimpleActionClient connected to the rogue server.
        :param patience_timeout: Duration to wait for the action server to acknowledge
                                 preemption before forcing goal cancellation.
        :returns: True if the action completes successfully, False otherwise.
        """
        # Send the goal to the action server
        action_client.send_goal(goal)

        loop_period = rospy.Duration(0.1)  # Wake up every 0.1 seconds to check preemption

        with action_client.done_condition:
            while not rospy.is_shutdown():
                # Wait with a timeout to periodically check preemption
                action_client.done_condition.wait(loop_period.to_sec())

                # Check if the goal has been completed
                if action_client.simple_state == actionlib.SimpleGoalState.DONE:
                    rospy.loginfo("Action completed successfully.")
                    return True

                # Check if user requested preemption
                if self.action_server.is_preempt_requested():
                    rospy.logwarn(
                        f'grasplan {self.action_server.action_server.ns} ' 'action server goal cancel request received'
                    )
                    action_client.cancel_goal()
                    break

        # Handle post-loop actions (preemption acknowledgment)
        if action_client.simple_state != actionlib.SimpleGoalState.DONE:
            rospy.loginfo(f"Waiting for {patience_timeout} seconds for MoveIt action server acknowledgment...")
            with action_client.done_condition:
                action_client.done_condition.wait(rospy.Duration(patience_timeout).to_sec())

            # If still not acknowledged, force preemption
            if action_client.simple_state != actionlib.SimpleGoalState.DONE:
                rospy.logwarn(
                    "MoveIt action server did not acknowledged preemption within "
                    f"{patience_timeout} s. Forcing preemption..."
                )
                self._force_preemption(action_client)
                return False

        rospy.logwarn("Action failed to complete successfully.")
        return False

    def _force_preemption(self, rogue_client, max_retries=100):
        """
        Repeatedly sends cancellation requests directly to joint trajectory controllers
        (bypassing MoveIt) until the rogue action server gives up and aborts.

        :param rogue_client: The client connected to the rogue server.
        :param max_retries: To guard against infinite loops
        """
        count = 1
        while rogue_client.get_state() != GoalStatus.ABORTED:
            rospy.loginfo(f'Force canceling joint trajectory goals at 10 Hz until aborted, attempt {count}')
            for canceller in self.canceller_list:
                if canceller.has_active_goals():
                    canceller.cancel_all_goals()
            rospy.sleep(0.1)  # Short wait before retrying
            count += 1

            if count > max_retries:
                rospy.logerr("Exceeded maximum retries for forcing preemption.")
                break
