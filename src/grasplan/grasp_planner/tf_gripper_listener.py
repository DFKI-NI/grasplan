#!/usr/bin/env python3

import time
import rospy
import tf

# function taken from : https://stackoverflow.com/questions/3136059/getting-one-value-from-a-tuple
# reads a single key pressed
def read_single_keypress():
    """Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns a tuple of characters of the key that was pressed - on Linux,
    pressing keys like up arrow results in a sequence of characters. Returns
    ('\x03',) on KeyboardInterrupt which can happen when a signal gets
    handled.

    """
    import termios, fcntl, sys, os

    fd = sys.stdin.fileno()
    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save)  # copy the stored version to update
    # iflag
    attrs[0] &= ~(
        termios.IGNBRK
        | termios.BRKINT
        | termios.PARMRK
        | termios.ISTRIP
        | termios.INLCR
        | termios.IGNCR
        | termios.ICRNL
        | termios.IXON
    )
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios.PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
    # read a single keystroke
    ret = []
    try:
        ret.append(sys.stdin.read(1))  # returns a single character
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save | os.O_NONBLOCK)
        c = sys.stdin.read(1)  # returns a single character
        while len(c) > 0:
            ret.append(c)
            c = sys.stdin.read(1)
    except KeyboardInterrupt:
        ret.append('\x03')
    finally:
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
    return tuple(ret)


class TFGripperListener:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.file_path = rospy.get_param('~file_path', '/tmp/grasp_transforms.yaml')
        self.end_effector_link = rospy.get_param('~end_effector_link', 'hand_ee_link')
        self.object_ref_frame = rospy.get_param('~object_ref_frame', 'tall_insole')

    def write_to_file(self, list_of_strings):
        if len(list_of_strings) == 3:
            rospy.logwarn('grasp poses file will not be generated (user did not pressed enter at least once)')
            return
        f = open(self.file_path, 'w+')
        for string in list_of_strings:
            f.write(string + '\n')
        f.close()

    def start(self):
        stream_list = ['# this file was generated automatically by tf_gripper_listener node']
        tab = '  '
        stream_list.append(f'{self.object_ref_frame}:')
        stream_list.append(f'{tab}grasp_poses:')
        rospy.sleep(3.0)
        while not rospy.is_shutdown():
            rospy.loginfo('Press Enter to record grasp... or q to quit and save')
            key = read_single_keypress()
            if key[0] == 'q' or key[0] == 'Q':
                break
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.object_ref_frame, self.end_effector_link, rospy.Time(0)
                )
                stream_list.append(f'{tab}{tab}-')
                # translation
                translation_str = f'{tab}{tab}{tab}translation: [{trans[0]:.6f}, {trans[1]:.6f}, {trans[2]:.6f}]'
                rospy.loginfo('\n')
                rospy.loginfo(translation_str)
                stream_list.append(translation_str)
                # rotation
                rotation_str = f'{tab}{tab}{tab}rotation: [{rot[0]:.6f}, {rot[1]:.6f}, {rot[2]:.6f}, {rot[3]:.6f}]'
                rospy.loginfo(rotation_str)
                stream_list.append(rotation_str)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr(
                    f'LookupTransform failed between frames: {self.object_ref_frame} and {self.end_effector_link}'
                )
        rospy.loginfo(f'writing to file: {self.file_path}')
        self.write_to_file(stream_list)


if __name__ == '__main__':
    rospy.init_node('tf_listener')
    tf_gripper_listener = TFGripperListener()
    tf_gripper_listener.start()
