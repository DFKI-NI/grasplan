#!/usr/bin/python3

import copy

import rospy
import tf
from dynamic_reconfigure.server import Server
from grasplan.cfg import objBoundsConfig

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from grasplan.tools.common import separate_object_class_from_id

from cob_perception_msgs.msg import Detection, DetectionArray

import tf2_ros
import tf2_geometry_msgs

class ObjRecognitionMockup:
    '''
    Define a virtual box with position and dimensions that represents the field of view of a camera
    Query Gazebo for all objects in the scene
    Iterate over all objects, see if they are inside the box, if so then publish their pose as cob_perception_msgs/DetectionArray
    Publish the object detections as tf
    This node can be used as a mockup for object 6D pose estimation and classification
    '''

    def __init__(self, test_pose=False):
        # get object bounding box from parameter
        self.bounding_boxes = rospy.get_param('~bounding_boxes', [])
        self.supress_warnings = rospy.get_param('~supress_warnings', False)
        self.broadcast_object_tf = rospy.get_param('~broadcast_object_tf', True)
        # the reference frame in which you desire the objects to be in
        self.objects_desired_reference_frame = rospy.get_param('~objects_desired_reference_frame', 'map')
        if self.supress_warnings:
            rospy.loginfo('NOTE: warnings for fake recognition node are supressed!')
        # a box representing an approximation of the fov of the camera
        self.perception_fov_pub = rospy.Publisher('~cube_fov', Marker, queue_size=1)
        self.test_pose = test_pose
        if self.test_pose:
            # pose to test if is_pose_inside_box is working correctly
            self.test_pose_pub = rospy.Publisher('~test_pose', PoseStamped, queue_size=1)
        # object recognition detections
        self.object_recognition_pub = rospy.Publisher('~detections', DetectionArray, queue_size=1)
        # response to trigger event
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        # subscribe to gazebo model states (gives pose of all existing objects in the simulation
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStatesCB)
        rospy.Subscriber('~event_in', String, self.eventInCB)
        # to store the mode states msg, coming from callback
        self.model_states_msg = None
        self.model_states_received = False
        # contains the latest state of the fov bounding box
        self.marker_msg = None
        rospy.loginfo('object recognition mockup node started')
        self.tf_broadcaster = tf.TransformBroadcaster()
        # to transform the object pose into another reference frame (self.objects_desired_reference_frame)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # keep track of the most approximate gazebo gt timestamp
        self.model_states_timestamp = rospy.Time.now()

    def modelStatesCB(self, msg):
        # gazebo model states does not publish timestamp, so this is the best we can do
        self.model_states_timestamp = rospy.Time.now()
        self.model_states_msg = msg
        self.model_states_received = True

    def transform_pose(self, input_pose, input_pose_reference_frame, target_frame_id):
        '''
        transform an input pose from a reference frame to another
        '''
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = input_pose_reference_frame
        pose_stamped_msg.header.stamp = self.model_states_timestamp
        pose_stamped_msg.pose.position.x = input_pose.position.x
        pose_stamped_msg.pose.position.y = input_pose.position.y
        pose_stamped_msg.pose.position.z = input_pose.position.z
        pose_stamped_msg.pose.orientation.x = input_pose.orientation.x
        pose_stamped_msg.pose.orientation.y = input_pose.orientation.y
        pose_stamped_msg.pose.orientation.z = input_pose.orientation.z
        pose_stamped_msg.pose.orientation.w = input_pose.orientation.w

        try:
            transform = self.tf_buffer.lookup_transform(
                                       # target reference frame
                                       target_frame_id,
                                       # source frame:
                                       input_pose_reference_frame,
                                       # get the tf at the time the pose was valid
                                       pose_stamped_msg.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))
        except:
            rospy.logerr(f'could not transform pose from {input_pose_reference_frame} to {target_frame_id}')
            return None

        return tf2_geometry_msgs.do_transform_pose(pose_stamped_msg, transform)

    def eventInCB(self, msg):
        rospy.loginfo('perceiving objects now (with mockup node)')
        if not self.model_states_received:
            rospy.logerr('Object perception mockup failed, have not received gazebo model states')
            self.event_out_pub.publish(String('e_not_found'))
            return
        detections_msg = DetectionArray()
        detections_msg.header.frame_id = self.objects_desired_reference_frame
        detections_msg.header.stamp = self.model_states_timestamp
        # iterate over all gazebo models
        for i, obj_pose in enumerate(self.model_states_msg.pose):
            model_name = self.model_states_msg.name[i]
            pose_msg = self.transform_pose(obj_pose, 'world', self.objects_desired_reference_frame)
            if not pose_msg:
                rospy.logerr('Object perception mockup failed, transform error')
                return
            if self.is_pose_inside_box(pose_msg, self.marker_msg):
                detection = Detection()
                detection.label = model_name
                detection.detector = 'mockup detector'
                detection.score = 1.0
                detection.pose.header.frame_id = detections_msg.header.frame_id
                detection.pose.header.stamp = detections_msg.header.stamp
                detection.pose.pose = copy.deepcopy(pose_msg.pose)

                # get bounding box from parameter yaml file
                object_class = separate_object_class_from_id(model_name)[0] # get object class from anchored object
                if self.bounding_boxes:
                    if object_class in self.bounding_boxes:
                        detection.bounding_box_lwh.x = self.bounding_boxes[object_class]['box_x']
                        detection.bounding_box_lwh.y = self.bounding_boxes[object_class]['box_y']
                        detection.bounding_box_lwh.z = self.bounding_boxes[object_class]['box_z']
                    else:
                        if not self.supress_warnings:
                            rospy.logwarn(f'object_class: {object_class} bounding box not found in parameters, will leave bb empty')
                        detection.bounding_box_lwh.x = 0.0
                        detection.bounding_box_lwh.y = 0.0
                        detection.bounding_box_lwh.z = 0.0
                else:
                    if not self.supress_warnings:
                        rospy.logwarn('bounding_boxes parameter not set')

                detections_msg.detections.append(detection)
                if self.broadcast_object_tf:
                    # broadcast object tf
                    self.tf_broadcaster.sendTransform((detection.pose.pose.position.x, detection.pose.pose.position.y, detection.pose.pose.position.z),
                        (detection.pose.pose.orientation.x, detection.pose.pose.orientation.y, detection.pose.pose.orientation.z, detection.pose.pose.orientation.w),
                        detection.pose.header.stamp, self.model_states_msg.name[i], self.objects_desired_reference_frame)

        if len(detections_msg.detections) == 0:
            if not self.supress_warnings:
                rospy.logwarn('no objects found')
            self.event_out_pub.publish(String('e_not_found'))
            return
        self.object_recognition_pub.publish(detections_msg)
        self.event_out_pub.publish(String('e_found'))
        rospy.loginfo('objects found!')

    def publish_test_pose(self, config):
        '''
        the test pose is taken from dynamic dynamic_reconfigure params
        '''
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.objects_desired_reference_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = config['test_pose_x']
        pose_msg.pose.position.y = config['test_pose_y']
        pose_msg.pose.position.z = config['test_pose_z']
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.test_pose_pub.publish(pose_msg)
        return pose_msg

    def publish_perception_fov(self, config):
        '''
        publish a cube in rviz as visualization marker to represent the fov of the camera
        this is done for object recognition mockup purposes
        '''
        marker_msg = Marker()
        marker_msg.header.frame_id = self.objects_desired_reference_frame
        marker_msg.type = Marker.CUBE
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = config['box_width']
        marker_msg.scale.y = config['box_length']
        marker_msg.scale.z = config['box_height']
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 0.5 # transparency
        marker_msg.pose.position.x = config['x_box_position']
        marker_msg.pose.position.y = config['y_box_position']
        marker_msg.pose.position.z = config['z_box_position']
        yaw = config['yaw_box_orientation']
        angular_q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        marker_msg.pose.orientation.x = angular_q[0]
        marker_msg.pose.orientation.y = angular_q[1]
        marker_msg.pose.orientation.z = angular_q[2]
        marker_msg.pose.orientation.w = angular_q[3]
        self.perception_fov_pub.publish(marker_msg)
        return marker_msg

    def is_pose_inside_box(self, pose_msg, marker_msg):
        if not marker_msg:
            rospy.logerr('error: marker msg must not be empty')
            return False
        pose_x = pose_msg.pose.position.x
        pose_y = pose_msg.pose.position.y
        pose_z = pose_msg.pose.position.z
        x1 = marker_msg.pose.position.x - marker_msg.scale.x / 2.0
        x2 = marker_msg.pose.position.x + marker_msg.scale.x / 2.0
        y1 = marker_msg.pose.position.y - marker_msg.scale.y / 2.0
        y2 = marker_msg.pose.position.y + marker_msg.scale.y / 2.0
        z1 = marker_msg.pose.position.z - marker_msg.scale.z / 2.0
        z2 = marker_msg.pose.position.z + marker_msg.scale.z / 2.0
        if x1 < pose_x and pose_x < x2:
            if y1 < pose_y and pose_y < y2:
                if z1 < pose_z and pose_z < z2:
                    return True
        return False

    def test_pose_method(self, config):
        pose_msg = self.publish_test_pose(config)
        if self.is_pose_inside_box(pose_msg, self.marker_msg):
            rospy.loginfo('object is in fov!')
        else:
            rospy.logerr('object is not in fov!')

    def reconfigureCB(self, config, level):
        self.marker_msg = self.publish_perception_fov(config)
        # test the publication of a pose defined in dynamic dynamic_reconfigure from within this node to see if the node logic is working
        if self.test_pose:
            self.test_pose_method(config)
        return config

    def start_object_recognition(self):
        srv = Server(objBoundsConfig, self.reconfigureCB)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_recognition', anonymous=False)
    object_recognition = ObjRecognitionMockup(test_pose=False)
    object_recognition.start_object_recognition()
