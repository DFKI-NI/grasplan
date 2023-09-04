#!/usr/bin/env python3

'''
publish all msgs inside a rosbag within a custom time interval
while doing so it replaces all msg timestamps with current time, so no --clock is necessary
however this means that you cannot rely on the time information for any calculation
'''

import rospy
import rosbag
import yaml
import importlib

class RosbagIntervalPub:
    def __init__(self, bag_path, topics_of_interest=[]):
        self.bag = rosbag.Bag(bag_path)
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.SafeLoader)
        self.bag_start = info_dict['start']
        self.bag_end = info_dict['end']
        self.bag_duration = info_dict['duration']
        rospy.loginfo(f'bag loaded, duration : {self.bag_duration}, start: {self.bag_start}, end: {self.bag_end}')

        self.topics_of_interest = topics_of_interest

        # setup publishers
        self.rosbag_pubs_dic = {} # topic name vs publisher
        if self.topics_of_interest == []:
            # all topics
            for topic in info_dict['topics']:
                c = getattr(importlib.import_module(topic['type'].split('/')[0]+'.msg'), topic['type'].split('/')[1])
                self.rosbag_pubs_dic[topic['topic']] = rospy.Publisher(topic['topic'], c, queue_size=1, latch=True)
        else:
            # selected topics
            for topic in info_dict['topics']:
                if topic['topic'] in self.topics_of_interest:
                    c = getattr(importlib.import_module(topic['type'].split('/')[0]+'.msg'), topic['type'].split('/')[1])
                    self.rosbag_pubs_dic[topic['topic']] = rospy.Publisher(topic['topic'], c, queue_size=1, latch=True)

        rospy.loginfo('init complete')

    def __del__(self):
        print('RosbagIntervalPub destructor called successfully')
        self.bag.close()

    def fix_msg_time(self, msg):
        if hasattr(msg, 'header'):
            msg.header.stamp = rospy.Time.now()
        return msg

    def pub_within_interval(self, start_time, end_time):
        '''
        start_time, end_time in secs
        '''
        t_start_time = rospy.Time.from_sec(start_time)
        t_end_time = rospy.Time.from_sec(end_time)
        rate = rospy.Rate(200)  # Publish rate in Hz
        if self.topics_of_interest == []:
            # all topics
            bag_iterator = iter(self.bag.read_messages(start_time=t_start_time, end_time=t_end_time))
        else:
            bag_iterator = iter(self.bag.read_messages(topics=self.topics_of_interest, start_time=t_start_time, end_time=t_end_time))

        for topic, msg, t in bag_iterator:
            # print(f'publishing on topic: {topic}')
            # remove
            if topic == '/tf' or topic == '/tf_static':
                for tf in msg.transforms:
                    self.fix_msg_time(tf)

            self.rosbag_pubs_dic[topic].publish(self.fix_msg_time(msg))
            rate.sleep()
            if rospy.is_shutdown():
                break

    def pub_within_percentage_interval(self, percentage_start, percentage_end):
        # bag_duration -> 100 %
        # x?           -> percentage_start
        start_time = percentage_start * self.bag_duration / 100.0 + self.bag_start
        end_time = percentage_end * self.bag_duration / 100.0 + self.bag_start
        rospy.loginfo(f'playing msgs within time interval : {start_time} - {end_time} [sec] ')
        self.pub_within_interval(start_time, end_time)

if __name__ == '__main__':
    rospy.init_node('rosbag_utils_tester')
    bag_path = '/home/oscar/Documents/mobipick/pbr_tables_demo/cic_demo_grasplan_config/2023-04-25-16-45-59.bag'
    rip = RosbagIntervalPub(bag_path)
    #rip.pub_within_interval(rip.bag_start, rip.bag_end)
    rip.pub_within_percentage_interval(0.0, 50.0)
