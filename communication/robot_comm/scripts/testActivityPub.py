#!/usr/bin/env python
import rospy
from comm_msg.msg import activity

def testPub():
    pub = rospy.Publisher('activity', activity, queue_size=10)
    rospy.init_node('testPubActivity', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        activity_msg  = activity()
        activity_msg.nb_robot = 1
        activity_msg.state = activity_msg.IN_PROGRESS
        activity_msg.machine_used = activity_msg.RS1
        activity_msg.nb_order = 10
        pub.publish(activity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        testPub()
    except rospy.ROSInterruptException:
        pass
