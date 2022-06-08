#!/usr/bin/env python3

import tf
import copy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class GraspEditorState:
    def __init__(self):
        self.__grasps = None
        self.__selected_grasp_index = None

    def set_grasps(self, grasps):
        assert isinstance(grasps, list)
        if len(grasps) == 0:
            self.__grasps = []
        else:
            assert isinstance(grasps[0], Pose)
            self.__grasps = copy.deepcopy(grasps) # deepcopy is essential here!

    def get_grasps(self):
        return self.__grasps

    def set_selected_grasp_index(self, selected_grasp_index):
        assert isinstance(selected_grasp_index, int)
        self.__selected_grasp_index = copy.deepcopy(selected_grasp_index)

    def get_selected_grasp_index(self):
        return self.__selected_grasp_index

class Grasps:
    def __init__(self, reference_frame='object', history_buffer_size=100):
        self.reference_frame = reference_frame
        self.history_buffer_size = history_buffer_size # the maximum number of times you can perform undo
        self.grasp_history = None
        self.undo_index = None
        self.__pause_history = None
        self.grasps_as_pose_array = None
        # flag used to highlight a grasp in green color, set to -1 to not highlight any pose in particular
        self.selected_grasp_index = None
        self.__init() # must be called at the end of this constructor

    def __init(self):
        self.grasps_as_pose_array = PoseArray()
        self.grasps_as_pose_array.header.frame_id = self.reference_frame
        self.__pause_history = False
        self.selected_grasp_index = -1
        self.undo_index = -1
        self.grasp_history = []
        for i in range(self.history_buffer_size):
            self.grasp_history.append(None)
        self.add_state_to_history()

    def pause_history(self):
        self.__pause_history = True

    def unpause_history(self):
        self.__pause_history = False

    def get_current_state(self):
        state = GraspEditorState()
        state.set_grasps(self.grasps_as_pose_array.poses)
        state.set_selected_grasp_index(self.selected_grasp_index)
        return state

    def add_state_to_history(self):
        '''
        for undo/redo purposes
        '''
        if not self.__pause_history:
            self.undo_index += 1
            if self.undo_index < self.history_buffer_size:
                self.grasp_history[self.undo_index] = self.get_current_state()
            else:
                # history_buffer_size exceeded
                del self.grasp_history[0]
                self.grasp_history.append(self.get_current_state())
                self.undo_index -= 1

    def add_grasp(self, grasp):
        assert isinstance(grasp, Pose)
        self.grasps_as_pose_array.poses.append(grasp)
        self.add_state_to_history()

    def add_grasps(self, pose_array_grasps):
        assert isinstance(pose_array_grasps, PoseArray)
        for grasp in pose_array_grasps.poses:
            self.add_grasp(grasp)

    def rotate_grasp(self, grasp, roll=0., pitch=0., yaw=0.):
        return self.transform_grasp(grasp, angular_rpy=[roll, pitch, yaw])

    def transform_grasp(self, grasp, linear=[0., 0., 0.], angular_rpy=[0., 0., 0.]):
        '''
        input: geometry_msgs/Pose (grasp) and the rotations in roll pitch yaw (in radians) in that order that you want to apply
        output: this function modifies grasp by reference
        ---
        quaternion rotation is applied via multiplication, see:
        http://wiki.ros.org/tf2/Tutorials/Quaternions , section "Applying a quaternion rotation"
        '''
        assert isinstance(grasp, Pose)
        assert isinstance(linear, list)
        assert isinstance(angular_rpy, list)
        q_orig = np.array([grasp.orientation.x, grasp.orientation.y, grasp.orientation.z, grasp.orientation.w])
        angular_q = tf.transformations.quaternion_from_euler(angular_rpy[0], angular_rpy[1], angular_rpy[2])
        q_new = tf.transformations.quaternion_multiply(angular_q, q_orig)
        grasp.position.x += linear[0]
        grasp.position.y += linear[1]
        grasp.position.z += linear[2]
        grasp.orientation.x = q_new[0]
        grasp.orientation.y = q_new[1]
        grasp.orientation.z = q_new[2]
        grasp.orientation.w = q_new[3]
        return grasp

    def rotate_grasps(self, grasps, roll=0., pitch=0., yaw=0., replace=False):
        self.transform_grasps(grasps, angular_rpy=[roll, pitch, yaw], replace=replace)

    def transform_grasps(self, grasps, linear=[0., 0., 0.], angular_rpy=[0., 0., 0.], replace=False):
        assert isinstance(grasps, list)
        self.pause_history() # for undo to work on all pattern poses we pause history
        static_grasps = copy.deepcopy(grasps)
        for grasp in static_grasps:
            assert isinstance(grasp, Pose)
            derived_grasp = copy.deepcopy(grasp)
            derived_grasp = self.transform_grasp(grasp, linear, angular_rpy)
            if replace:
                self.remove_selected_grasp()
            self.add_grasp(derived_grasp)
        self.unpause_history() # for undo to work on all pattern poses we unpause history

    def rotate_selected_grasps(self, roll=0., pitch=0., yaw=0., replace=False):
        return self.transform_selected_grasps(angular_rpy=[roll, pitch, yaw], replace=replace)

    def transform_selected_grasps(self, linear=[0., 0., 0.], angular_rpy=[0., 0., 0.], replace=False):
        if self.no_grasp_is_selected():
            return False
        grasps = self.get_selected_grasps()
        assert isinstance(grasps, list)
        if grasps == []:
            return False
        else:
            self.transform_grasps(grasps, linear, angular_rpy, replace)
            return True

    def remove_grasp(self, grasp):
        assert isinstance(grasp, Pose)
        self.grasps_as_pose_array.poses.remove(grasp)
        self.add_state_to_history()

    def remove_grasp_by_index(self, grasp_index):
        assert isinstance(grasp_index, int)
        if grasp_index == -10:
            self.remove_all_grasps()
        elif grasp_index != -1:
            grasp = self.grasps_as_pose_array.poses[grasp_index]
            self.remove_grasp(grasp)

    def remove_selected_grasp(self):
        if self.all_grasps_are_selected():
            self.remove_all_grasps()
        else:
            self.remove_grasp_by_index(self.selected_grasp_index)
        self.unselect_all_grasps()

    def remove_selected_grasps(self):
        self.remove_selected_grasp()

    def remove_all_but_one_grasp(self):
        grasp_zero = copy.deepcopy(self.grasps_as_pose_array.poses[0])
        self.grasps_as_pose_array.poses = [grasp_zero]
        self.select_grasp(0)

    def remove_all_grasps(self):
        self.grasps_as_pose_array.poses = []
        self.unselect_all_grasps()

    def get_grasp_by_index(self, grasp_index):
        assert isinstance(grasp_index, int)
        return copy.deepcopy(self.grasps_as_pose_array.poses[grasp_index])

    def replace_grasp_by_index(self, grasp_index, new_grasp):
        assert isinstance(grasp_index, int)
        assert isinstance(new_grasp, Pose)
        self.grasps_as_pose_array.poses[grasp_index] = new_grasp
        self.add_state_to_history()

    def get_grasps_as_pose_list(self):
        return copy.deepcopy(self.grasps_as_pose_array.poses)

    def get_grasps_as_pose_array_msg(self):
        return self.grasps_as_pose_array

    def no_grasp_is_selected(self):
        '''
        selected_grasp_index = -1  # no grasp is selected
        selected_grasp_index = -10 # all grasps are selected
        selected_grasp_index = 4   # grasp no. 4 is selected
        '''
        if self.selected_grasp_index == -1:
            return True
        return False

    def all_grasps_are_selected(self):
        '''
        selected_grasp_index = -1  # no grasp is selected
        selected_grasp_index = -10 # all grasps are selected
        selected_grasp_index = 4   # grasp no. 4 is selected
        '''
        if self.selected_grasp_index == -10:
            return True
        return False

    def get_selected_grasp(self):
        return self.get_grasp_by_index(self.selected_grasp_index)

    def get_selected_grasps(self):
        if self.single_grasp_is_selected():
            return [self.get_selected_grasp()]
        elif self.no_grasp_is_selected():
            return []
        elif self.selected_grasp_index == -10: # all grasps are selected
            return self.get_grasps_as_pose_list()

    def get_selected_grasp_index(self):
        return self.selected_grasp_index

    def select_grasp(self, grasp_index):
        assert isinstance(grasp_index, int)
        if grasp_index > self.size() - 1:
            return False
        else:
            self.selected_grasp_index = grasp_index
            self.add_state_to_history()
            return True

    def select_all_grasps(self):
        self.selected_grasp_index = -10
        self.add_state_to_history()

    def select_last_grasp(self):
        self.selected_grasp_index = self.size() - 1
        self.add_state_to_history()

    def unselect_all_grasps(self):
        self.selected_grasp_index = -1
        self.add_state_to_history()

    def single_grasp_is_selected(self):
        if self.all_grasps_are_selected() or self.no_grasp_is_selected():
            return False
        else:
            return True

    def size(self):
        return len(self.grasps_as_pose_array.poses)

    def restore_state(self):
        state = self.grasp_history[self.undo_index]
        self.grasps_as_pose_array.poses = state.get_grasps()
        self.selected_grasp_index = state.get_selected_grasp_index()

    def undo(self):
        if self.undo_index - 1 < 0:
            return "can't undo any further"
        else:
            self.undo_index -= 1
            self.restore_state()

    def redo(self):
        if self.undo_index + 1 < self.history_buffer_size:
            if self.grasp_history[self.undo_index + 1] is None:
                return "can't redo any further, already at the latest action"
            else:
                self.undo_index += 1
                self.restore_state()
        else:
            return "can't redo any further"
