#!/usr/bin/env python3

'''
run command via subprocess:

  rosbag play my_bag.bag --start x --duration y.

allows to easily set x and y parameters by specifying a percentage or interval that you want to play
'''

import rospy
import rosbag
import yaml
import subprocess


class RosbagIntervalPub:
    '''
    # Usage example:

    if __name__ == '__main__':
        rospy.init_node('rosbag_utils_tester')
        bag_path = '/home/user/my_bag.bag'
        rip = RosbagIntervalPub(bag_path)

        # play all msgs within 45 and 50 percent of the rosbag
        # e.g. rosbag duration = 100.0 secs, will play all msgs between 45.0 and 50.0 secs
        rip.pub_within_percentage_interval(45.0, 50.0)
    '''

    def __init__(self, bag_path):
        self.bag = rosbag.Bag(bag_path)
        self.bag_path = bag_path
        # get metadata from bag
        info_dict = yaml.load(self.bag._get_yaml_info(), Loader=yaml.SafeLoader)
        self.bag_start = info_dict['start']
        self.bag_end = info_dict['end']
        self.bag_duration = info_dict['duration']
        rospy.loginfo(f'bag loaded, duration : {self.bag_duration}, start: {self.bag_start}, end: {self.bag_end}')
        rospy.loginfo('init complete')

    def __del__(self):
        rospy.loginfo('RosbagIntervalPub destructor called successfully')
        self.bag.close()

    def pub_within_interval(self, start_time, end_time):
        '''
        start_time, end_time in seconds
        '''
        duration = end_time - start_time
        subprocess_args = [
            'rosbag',
            'play',
            str(self.bag_path),
            '--start',
            str(start_time - self.bag_start),
            '--duration',
            str(duration),
        ]
        cmd_str = ''
        for arg in subprocess_args:
            cmd_str += arg + ' '
        rospy.loginfo(f'running command: {cmd_str}')
        subprocess.run(subprocess_args)

    def pub_within_percentage_interval(self, percentage_start, percentage_end):
        '''
        bag_duration -> 100 %
        x?           -> percentage_start
        '''
        start_time = percentage_start * self.bag_duration / 100.0 + self.bag_start
        end_time = percentage_end * self.bag_duration / 100.0 + self.bag_start
        rospy.loginfo(f'playing msgs within time interval : {start_time} - {end_time} [sec] ')
        self.pub_within_interval(start_time, end_time)
