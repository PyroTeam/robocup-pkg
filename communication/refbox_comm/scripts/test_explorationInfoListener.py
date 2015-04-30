#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('refBoxComm')
from comm_msg.msg import ExplorationInfo
from comm_msg.msg import LightSpec

def stringColor(color):
    if color == LightSpec.RED :
        return "RED"
    elif color == LightSpec.YELLOW :
        return "YELLOW"
    elif color == LightSpec.GREEN :
        return "GREEN"
    return ""
   
def stringState(state):
    if state == LightSpec.OFF :
        return "OFF"
    elif state == LightSpec.ON :
        return "ON "
    elif state == LightSpec.BLINK :
        return "BLINK"
    return ""
   
def callback(ei):
    rospy.loginfo("%s : Declaration des types et machines", rospy.get_caller_id())

    for signal in ei.signals:
        strSignal = signal.type + " : "
        for light in signal.lights:
            strSignal += stringColor(light.color)
            strSignal += " = "
            strSignal += stringState(light.state)
            strSignal += " | "
        rospy.loginfo(strSignal) 

    for machine in ei.machines:
        rospy.loginfo("%s : (%.3f, %.3f, %.3f)", machine.name, machine.pose.x, machine.pose.x, machine.pose.theta)


    
def ei_listener():
    rospy.init_node('test_explorationInfoListener', anonymous=True)

    rospy.Subscriber("/refBoxComm/ExplorationInfo", ExplorationInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    ei_listener()



