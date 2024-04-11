import tf2_ros
import moveit_commander
import rospy
import sys


class GraspPlanBase:
    def __init__(self) -> None:
        self.planning_time = rospy.get_param("~planning_time", 20.0)
        self.arm_goal_tolerance = rospy.get_param("~arm_goal_tolerance", 0.01)
        self.global_reference_frame = rospy.get_param("~global_reference_frame", 'map')

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        rospy.sleep(3)  # TODO: Why do we need to sleep?
        # TODO: Is this the right way to get the gripper?
        self.gripper = getattr(self.robot, "gripper")
        self.scene = moveit_commander.PlanningSceneInterface()

        self.robot.arm.set_planning_time(self.planning_time)
        self.robot.arm.set_goal_tolerance(self.arm_goal_tolerance)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
