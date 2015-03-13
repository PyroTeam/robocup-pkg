#!/usr/bin/env python
import roslib; roslib.load_manifest('refbox_comm')

import sys

import rospy
from comm_msg.srv import ReportMachine
from std_msgs.msg import String

def reportMachine_client(n, t):
    rospy.wait_for_service('/refBoxComm/ReportMachine')
    try:
        report_machine = rospy.ServiceProxy('/refBoxComm/ReportMachine', ReportMachine)

        resp1 = report_machine(n, t)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [name type]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        n = str(sys.argv[1])
        t = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    rospy.loginfo("Request")
    print "Request report machine %s type %s"%(n, t)
    reportMachine_client(n, t)


