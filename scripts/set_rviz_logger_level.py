#!/usr/bin/python3

'''
set rviz logger level via srv call to suppress warnings e.g. from tf
'''

import rospy
from roscpp.srv import SetLoggerLevel

def main():
    rospy.init_node('set_rviz_logger_level')
    rospy.wait_for_service('/rviz/set_logger_level')
    try:
        set_level = rospy.ServiceProxy('/rviz/set_logger_level', SetLoggerLevel)
        resp = set_level('rviz', 'ERROR')
        rospy.logwarn("rviz logger level set to ERROR (to supress warnings)")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    main()
