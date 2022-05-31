#!/usr/bin/env python3

import os
import tf
import math
import copy
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from grasplan.common_grasp_tools import remove_object_id
from grasplan.visualisation.grasp_visualiser import GraspVisualiser

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

class RqtGrasplan(Plugin):

    def __init__(self, context):
        super(RqtGrasplan, self).__init__(context)
        rospy.loginfo('Initializing grasplan rqt, have a happy grasp editing !')

        self.setObjectName('RqtGrasplan')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('grasplan'), 'config/rqt_grasplan', 'rqtgrasplan.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqtgrasplan.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        rospy.loginfo('start widget connections...')

        ## init

        self.tab = '    ' # used in save function

        # parameters
        self.grasp_poses = rospy.get_param('~handcoded_grasp_planner_transforms')
        self.object_name = rospy.get_param('~object_name', 'multimeter')
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')

        # publications
        self.pose_highlight_pub = rospy.Publisher('/rviz_gripper_visualiser/highlight_pose', Int8, queue_size=1)
        self.grasp_poses_pub = rospy.Publisher('/grasp_editor/grasp_poses', PoseArray, queue_size=1)
        self.test_pose_pub = rospy.Publisher('/test_pose', PoseStamped, queue_size=1)

        # flags
        self.selected_pose = -1 # flag used to highlight a grasp in green color, set to -1 to not highlight any pose in particular

        # variable that stores all grasps
        self.grasps_as_pose_array = PoseArray()
        self.grasps_as_pose_array.header.frame_id = self.global_reference_frame

        # load grasps from param server
        # TODO: set proper parameter
        object_class = remove_object_id(self.object_name)
        if not object_class in self.grasp_poses:
            rospy.logerr(f'object "{object_class}" not found in dictionary, have you included in handcoded_grasp_planner_transforms parameter?')
        else:
            rospy.loginfo(f'loading {self.object_name} grasps from param server')
            for transform in self.grasp_poses[object_class]['grasp_poses']:
                pose_msg = Pose()
                pose_msg.position.x = transform['translation'][0]
                pose_msg.position.y = transform['translation'][1]
                pose_msg.position.z = transform['translation'][2]
                pose_msg.orientation.x = transform['rotation'][0]
                pose_msg.orientation.y = transform['rotation'][1]
                pose_msg.orientation.z = transform['rotation'][2]
                pose_msg.orientation.w = transform['rotation'][3]
                self.grasps_as_pose_array.poses.append(pose_msg)

        # publish object mesh to rviz with texture
        grasp_visualiser = GraspVisualiser()
        mesh_path = f'package://mobipick_gazebo/meshes/{self.object_name}.dae'
        marker_msg = grasp_visualiser.make_mesh_marker_msg(mesh_path)
        grasp_visualiser.marker_pub.publish(marker_msg)

        # visualise grasps at startup
        self.publish_grasps()

        ## make a connection between the qt objects and this class methods
        self._widget.cmdSave.clicked.connect(self.handle_save_button)
        self._widget.cmdGraspSSelect.clicked.connect(self.handle_grasp_s_select_button)
        self._widget.cmdGraspSDelete.clicked.connect(self.handle_grasp_s_delete_button)
        self._widget.cmdEditGApply.clicked.connect(self.handle_edit_g_apply_button)
        self._widget.cmdTransformApply.clicked.connect(self.handle_transform_apply_button)
        self._widget.cmdGraspSUnselect.clicked.connect(self.handle_grasp_s_unselect_button)
        self._widget.cmdTransformCreateGrasp.clicked.connect(self.handle_transform_create_grasp_button)
        self._widget.cmdTransformQ2RPY.clicked.connect(self.handle_transform_q_2_rpy_button)
        self._widget.cmdTransformRPY2Q.clicked.connect(self.handle_transform_rpy_2_q_button)
 
        # inform the user how many grasps were loaded
        self.update_grasp_number()
        context.add_widget(self._widget)
        rospy.loginfo('init finished')
        # end of constructor

    # ::::::::::::::  class methods

    def visualise_transform(self, translation=[0,0,0], rotation=[0,0,0,1]):
        # publish transform to test pose for visualisation purposes
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = self.global_reference_frame
        pose_stamped_msg.pose.position.x = translation[0]
        pose_stamped_msg.pose.position.y = translation[1]
        pose_stamped_msg.pose.position.z = translation[2]
        pose_stamped_msg.pose.orientation.x = rotation[0]
        pose_stamped_msg.pose.orientation.y = rotation[1]
        pose_stamped_msg.pose.orientation.z = rotation[2]
        pose_stamped_msg.pose.orientation.w = rotation[3]
        self.test_pose_pub.publish(pose_stamped_msg)

    def fix_displayed_text(self):
        if self._widget.txtTransformLinearX.toPlainText() == '-0.0':
            self._widget.txtTransformLinearX.setPlainText('0.0')
        if self._widget.txtTransformLinearY.toPlainText() == '-0.0':
            self._widget.txtTransformLinearY.setPlainText('0.0')
        if self._widget.txtTransformLinearZ.toPlainText() == '-0.0':
            self._widget.txtTransformLinearZ.setPlainText('0.0')
        if self._widget.txtTransformAngularR.toPlainText() == '-0.0':
            self._widget.txtTransformAngularR.setPlainText('0.0')
        if self._widget.txtTransformAngularP.toPlainText() == '-0.0':
            self._widget.txtTransformAngularP.setPlainText('0.0')
        if self._widget.txtTransformAngularY.toPlainText() == '-0.0':
            self._widget.txtTransformAngularY.setPlainText('0.0')

    def handle_transform_q_2_rpy_button(self):
        Qx = float(self._widget.txtTransformAngularQx.toPlainText())
        Qy = float(self._widget.txtTransformAngularQy.toPlainText())
        Qz = float(self._widget.txtTransformAngularQz.toPlainText())
        Qw = float(self._widget.txtTransformAngularQw.toPlainText())
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([Qx, Qy, Qz, Qw])
        # convert units to degrees
        if self._widget.optTransformUnitsDeg.isChecked():
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)
        self._widget.txtTransformAngularR.setPlainText(str(round(roll, 2)))
        self._widget.txtTransformAngularP.setPlainText(str(round(pitch, 2)))
        self._widget.txtTransformAngularY.setPlainText(str(round(yaw, 2)))
        # avoid -0.0 being displayed as text
        self.fix_displayed_text()
        # publish transform to rviz for visualisation purposes
        self.visualise_transform(rotation=[Qx, Qy, Qz, Qw])

    def handle_transform_rpy_2_q_button(self):
        roll = float(self._widget.txtTransformAngularR.toPlainText())
        pitch = float(self._widget.txtTransformAngularP.toPlainText())
        yaw = float(self._widget.txtTransformAngularY.toPlainText())
        if self._widget.optTransformUnitsRad.isChecked():
            if roll > math.pi or pitch > math.pi or yaw > math.pi:
                rospy.logwarn('radians are selected but value is greater than pi, is this correct?')
        if self._widget.optTransformUnitsDeg.isChecked():
            # convert to radians
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        q_new = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self._widget.txtTransformAngularQx.setPlainText(str(round(q_new[0], 4)))
        self._widget.txtTransformAngularQy.setPlainText(str(round(q_new[1], 4)))
        self._widget.txtTransformAngularQz.setPlainText(str(round(q_new[2], 4)))
        self._widget.txtTransformAngularQw.setPlainText(str(round(q_new[3], 4)))
        # publish transform to rviz for visualisation purposes
        self.visualise_transform(rotation=q_new)

    def publish_grasps(self):
        self.pose_highlight_pub.publish(self.selected_pose)
        self.grasp_poses_pub.publish(self.grasps_as_pose_array)

    def handle_save_button(self):
        rospy.loginfo('save!')
        for grasp in self.grasps_as_pose_array.poses:
            print(f'{self.tab}-\n{self.tab}  translation: [{grasp.position.x},{grasp.position.y},{grasp.position.z}]\n' +\
                     f'{self.tab}  rotation: [{grasp.orientation.x},{grasp.orientation.y},{grasp.orientation.z},{grasp.orientation.w}]')
        # -
        #   translation: [-0.004005, -0.025508, -0.001361]
        #   rotation: [0.562853, 0.426008, 0.452734, 0.544743]

    def update_selected_pose(self):
        grasp_number = None
        if self._widget.chkGraspSAllGrasps.isChecked():
            self.selected_pose = -10 # -10 means select all poses
            return True
        else:
            grasp_number = int(self._widget.txtGraspSGraspNumbers.toPlainText())
            if grasp_number >= len(self.grasps_as_pose_array.poses):
                rospy.logerr('cannot select/highlight grasp, selected number is out of bounds')
                return False
            else:
                self.selected_pose = grasp_number
                return True

    def handle_grasp_s_select_button(self):
        rospy.loginfo('select!')
        if self.update_selected_pose():
            self.publish_grasps()
            # load selected pose in transform if needed
            if self._widget.chkTransformLoadSelected.isChecked():
                if self.selected_pose != -10 and self.selected_pose != -1:
                    selected_pose = self.grasps_as_pose_array.poses[self.selected_pose]
                    self._widget.txtTransformLinearX.setPlainText(str(round(selected_pose.position.x, 2)))
                    self._widget.txtTransformLinearY.setPlainText(str(round(selected_pose.position.y, 2)))
                    self._widget.txtTransformLinearZ.setPlainText(str(round(selected_pose.position.z, 2)))
                    # set quaternion
                    q = [selected_pose.orientation.x, selected_pose.orientation.y, selected_pose.orientation.z, selected_pose.orientation.w]
                    self._widget.txtTransformAngularQx.setPlainText(str(round(q[0], 4)))
                    self._widget.txtTransformAngularQy.setPlainText(str(round(q[1], 4)))
                    self._widget.txtTransformAngularQz.setPlainText(str(round(q[2], 4)))
                    self._widget.txtTransformAngularQw.setPlainText(str(round(q[3], 4)))
                    # rpy
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
                    if self._widget.optTransformUnitsDeg.isChecked():
                        # convert to degrees
                        roll = math.degrees(roll)
                        pitch = math.degrees(pitch)
                        yaw = math.degrees(yaw)
                    self._widget.txtTransformAngularR.setPlainText(str(round(roll, 2)))
                    self._widget.txtTransformAngularP.setPlainText(str(round(pitch, 2)))
                    self._widget.txtTransformAngularY.setPlainText(str(round(yaw, 2)))
                    self.fix_displayed_text()

    def update_grasp_number(self):
        self._widget.lblGraspSGrasps.setText(str(len(self.grasps_as_pose_array.poses)))

    def handle_grasp_s_delete_button(self):
        rospy.loginfo('delete!')
        if self._widget.chkGraspSAllGrasps.isChecked():
            if len(self.grasps_as_pose_array.poses) == 1:
                rospy.loginfo('deleting all poses!')
                self.grasps_as_pose_array.poses = []
            else:
                rospy.logwarn('deleting all poses but leaving pose 0, if you want to delete even pose 0 click delete again')
                pose0 = copy.deepcopy(self.grasps_as_pose_array.poses[0])
                self.grasps_as_pose_array.poses = []
                self.grasps_as_pose_array.poses.append(pose0)
            self.selected_pose = -1
            self.publish_grasps()
        elif self.update_selected_pose():
            del self.grasps_as_pose_array.poses[self.selected_pose]
            self.selected_pose = -1
            self.publish_grasps()
        self.update_grasp_number()

    def handle_edit_g_apply_button(self):
        rospy.loginfo('apply pattern!')
        # mirror
        if self._widget.optEditGPatternMirror.isChecked():
            static_grasps = copy.deepcopy(self.grasps_as_pose_array.poses)
            if self.update_selected_pose():
                if self.selected_pose == -10:
                    for grasp_pose in static_grasps:
                        derived_grasp = copy.deepcopy(grasp_pose)
                        q_orig = [derived_grasp.orientation.x, derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                        if self._widget.chkEditGAxisX.isChecked():
                            q_orig = [-derived_grasp.orientation.x, derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                        if self._widget.chkEditGAxisY.isChecked():
                            q_orig = [derived_grasp.orientation.x, -derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                        if self._widget.chkEditGAxisZ.isChecked():
                            q_orig = [derived_grasp.orientation.x, derived_grasp.orientation.y, -derived_grasp.orientation.z, derived_grasp.orientation.w]
                        derived_grasp.orientation.x = q_orig[0]
                        derived_grasp.orientation.y = q_orig[1]
                        derived_grasp.orientation.z = q_orig[2]
                        derived_grasp.orientation.w = q_orig[3]
                        if self._widget.optEditGHandlingCopyR.isChecked():
                            self.grasps_as_pose_array.poses.remove(grasp_pose)
                        # append new pose
                        self.grasps_as_pose_array.poses.append(derived_grasp)
                else:
                    derived_grasp = copy.deepcopy(self.grasps_as_pose_array.poses[self.selected_pose])
                    q_orig = [derived_grasp.orientation.x, derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                    if self._widget.chkEditGAxisX.isChecked():
                        q_orig = [-derived_grasp.orientation.x, derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                    if self._widget.chkEditGAxisY.isChecked():
                        q_orig = [derived_grasp.orientation.x, -derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                    if self._widget.chkEditGAxisZ.isChecked():
                        q_orig = [derived_grasp.orientation.x, derived_grasp.orientation.y, -derived_grasp.orientation.z, derived_grasp.orientation.w]
                    derived_grasp.orientation.x = q_orig[0]
                    derived_grasp.orientation.y = q_orig[1]
                    derived_grasp.orientation.z = q_orig[2]
                    derived_grasp.orientation.w = q_orig[3]
                    #replace pose
                    if self._widget.optEditGHandlingCopyR.isChecked():
                        del self.grasps_as_pose_array.poses[self.selected_pose]
                    # append new pose
                    self.grasps_as_pose_array.poses.append(derived_grasp)
            self.publish_grasps()
        # circular pattern
        if self._widget.optEditGPatternCircular.isChecked():
            # read angular step
            ang_step = float(self._widget.txtEditGAngStep.toPlainText())
            if self._widget.optEditGAngularUnitsDeg.isChecked():
                ang_step = math.radians(ang_step)
            if ang_step > math.pi:
                rospy.logwarn('step angle is greater than 360 deg (pi radians), is this correct?')
            # read number of times that the user wants to repeat the grasp
            number_of_grasps = int(self._widget.txtEditGNumberOfGrasps.toPlainText())
            if number_of_grasps < 0:
                rospy.logerr('number of grasps to make pattern cannot be negative')
                return
            # read selected pose and check if it is valid
            if self.update_selected_pose():
                parent_grasp = self.grasps_as_pose_array.poses[self.selected_pose]
                for pattern_grasp in range(number_of_grasps - 1):
                    derived_grasp = copy.deepcopy(parent_grasp)
                    q_orig = [derived_grasp.orientation.x, derived_grasp.orientation.y, derived_grasp.orientation.z, derived_grasp.orientation.w]
                    roll, pitch, yaw = [0, 0, 0]
                    if self._widget.chkEditGAxisX.isChecked():
                        yaw -= ang_step
                    if self._widget.chkEditGAxisY.isChecked():
                        pitch += ang_step
                    if self._widget.chkEditGAxisZ.isChecked():
                        roll += ang_step
                    q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                    # multiply quaternions (apply rotation), see: http://wiki.ros.org/tf2/Tutorials/Quaternions
                    q_new = tf.transformations.quaternion_multiply(q_rot, q_orig)
                    derived_grasp.orientation.x = q_new[0]
                    derived_grasp.orientation.y = q_new[1]
                    derived_grasp.orientation.z = q_new[2]
                    derived_grasp.orientation.w = q_new[3]
                    # append new pose
                    self.grasps_as_pose_array.poses.append(derived_grasp)
                    # prepare for next pose
                    parent_grasp = copy.deepcopy(derived_grasp)
                    # highlight and select newly generated grasp
                    self.selected_pose = len(self.grasps_as_pose_array.poses) - 1
                self.publish_grasps()
        self.update_grasp_number()

    def animate_rotation(self):
        # this function is not really used but is left here for the time being...
        #translation = [-0.001965, 0.000128, 0.017194]
        translation = [0.0, 0.0, 0.0]
        #rotation = [0.026126, 0.706826, -0.009555, 0.706840]
        rotation = [0.0, 0.0, 0.0, 1.0]
        #---
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = 'map'
        pose_stamped_msg.pose.position.x = translation[0]
        pose_stamped_msg.pose.position.y = translation[1]
        pose_stamped_msg.pose.position.z = translation[2]
        pose_stamped_msg.pose.orientation.x = rotation[0]
        pose_stamped_msg.pose.orientation.y = rotation[1]
        pose_stamped_msg.pose.orientation.z = rotation[2]
        pose_stamped_msg.pose.orientation.w = rotation[3]
        # ---
        quaternion = [pose_stamped_msg.pose.orientation.x, pose_stamped_msg.pose.orientation.y, pose_stamped_msg.pose.orientation.z, pose_stamped_msg.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        for i in range(1, 100):
            if self._widget.chkEditGAxisX.isChecked():
                roll += 0.1
            if self._widget.chkEditGAxisY.isChecked():
                pitch += 0.1
            if self._widget.chkEditGAxisZ.isChecked():
                yaw += 0.1
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            pose_stamped_msg.pose.orientation.x = quaternion[0]
            pose_stamped_msg.pose.orientation.y = quaternion[1]
            pose_stamped_msg.pose.orientation.z = quaternion[2]
            pose_stamped_msg.pose.orientation.w = quaternion[3]
            rospy.sleep(0.1)
            self.test_pose_pub.publish(pose_stamped_msg)

    def build_pose_msg(self, linear, angular):
        pose_msg = Pose()
        pose_msg.position.x = linear[0]
        pose_msg.position.y = linear[1]
        pose_msg.position.z = linear[2]
        pose_msg.orientation.x = angular[0]
        pose_msg.orientation.y = angular[1]
        pose_msg.orientation.z = angular[2]
        pose_msg.orientation.w = angular[3]
        return pose_msg

    def handle_transform_apply_button(self):
        rospy.loginfo('apply transform!')
        if self.selected_pose == -10 or self.selected_pose == -1:
            # apply transform to all
            rospy.logwarn('not implemented yet :(')
        else:
            linear, angular = self.read_transform()
            pose_msg = self.build_pose_msg(linear, angular)
            self.grasps_as_pose_array.poses[self.selected_pose] = pose_msg
            self.publish_grasps()

    def handle_grasp_s_unselect_button(self):
        rospy.loginfo('unselect!')
        self.selected_pose = -1
        self.publish_grasps()

    def read_transform(self):
        return [float(self._widget.txtTransformLinearX.toPlainText()),\
            float(self._widget.txtTransformLinearY.toPlainText()),\
            float(self._widget.txtTransformLinearZ.toPlainText())],\
            [float(self._widget.txtTransformAngularQx.toPlainText()),\
            float(self._widget.txtTransformAngularQy.toPlainText()),\
            float(self._widget.txtTransformAngularQz.toPlainText()),\
            float(self._widget.txtTransformAngularQw.toPlainText())]

    def handle_transform_create_grasp_button(self):
        rospy.loginfo('create grasp!')
        linear, angular = self.read_transform()
        pose_msg = Pose()
        pose_msg.position.x = linear[0]
        pose_msg.position.y = linear[1]
        pose_msg.position.z = linear[2]
        pose_msg.orientation.x = angular[0]
        pose_msg.orientation.y = angular[1]
        pose_msg.orientation.z = angular[2]
        pose_msg.orientation.w = angular[3]
        self.grasps_as_pose_array.poses.append(pose_msg)
        self.selected_pose = len(self.grasps_as_pose_array.poses) - 1
        self.publish_grasps()
        self.update_grasp_number()
