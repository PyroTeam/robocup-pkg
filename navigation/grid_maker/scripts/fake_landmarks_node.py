#!/usr/bin/env python
# encoding: UTF-8

import sys
import yaml
import rospy
from geometry_msgs.msg import Pose2D
from deplacement_msg.msg import Landmarks


def load_machines(config, machines):
    machines.landmarks = []

    rawMachines = config["machines"]
    for machineName in rawMachines:
        machine = rawMachines[machineName]
        pose = machine["pose"]
        p = Pose2D()
        p.x = pose[0]
        p.y = pose[1]
        p.theta = pose[2]
        machines.landmarks.append(p)


def machine_publisher(config_file):
    data = yaml.load(config_file)
    machines_msg = Landmarks()

    machines_msg.header.frame_id = data["frame_id"]
    load_machines(data, machines_msg)

    machines_pub = rospy.Publisher('objectDetection/landmarks', Landmarks, queue_size=10)
    rospy.init_node('fake_landmarks', anonymous=False)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        machines_msg.header.seq = machines_msg.header.seq + 1
        machines_msg.header.stamp = rospy.Time.now()
        machines_pub.publish(machines_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        machine_publisher(open(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
