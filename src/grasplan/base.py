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

import tf2_ros
import moveit_commander
import rospy
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Transform, TransformStamped
from tf2_geometry_msgs import PoseStamped, PointStamped
from typing import List
from object_pose_msgs.msg import ObjectList


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

    def transform_point(self, point: Point, source_frame: str, target_frame: str) -> Point:
        """Transform a point from a source frame into a target frame"""
        point_stamped = PointStamped(Header(frame_id=source_frame), point)
        return self.tf_buffer.transform(point_stamped, target_frame).point

    def transform_points(self, points: List[Point], source_frame: str, target_frame: str) -> List[Point]:
        """Transform a list of points from a source frame into a target frame"""
        return [self.transform_point(point, source_frame, target_frame) for point in points]

    def transform_obj_list(self, obj_list: ObjectList, target_frame: str) -> ObjectList:
        """Transform an ObjectList to the target frame"""
        for obj in obj_list.objects:
            source_pose = PoseStamped(obj_list.header, obj.pose)
            target_pose = self.tf_buffer.transform(source_pose, target_frame)
            obj.pose = target_pose.pose
        obj_list.header.frame_id = target_frame
        return obj_list

    def add_scene_objs_to_tf(self) -> None:
        """Adds all objects in the planning scene to the tf tree as static transforms"""
        for obj in self.scene.get_objects().values():
            tf = TransformStamped(
                header=Header(frame_id=self.global_reference_frame),
                child_frame_id=obj.id,
                transform=Transform(translation=obj.pose.position, rotation=obj.pose.orientation),
            )
            self.tf_buffer.set_transform_static(tf, "grasplan")
