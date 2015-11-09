#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def input_publisher():
    pub = rospy.Publisher('baxter_input', String, queue_size = 10)
    rospy.init_node('input_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raw_input('Press enter to send something to RobotInterface')
        pub_str = 'data'
        pub.publish(pub_str)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        input_publisher()
    except rospy.ROSInterruptException:
        pass
