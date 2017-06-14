#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool

if __name__ == '__main__':

    pub = rospy.Publisher('stop_time', Bool, latch = True, queue_size = 10)

    rospy.init_node('Stopper')

    # num of minutes times number of seconds
    rospy.sleep(15 * 60)

    pub.publish(1)
