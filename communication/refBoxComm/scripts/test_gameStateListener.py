#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('refBoxComm')
from refBoxComm.msg import GameState

def callback(gs):
    rospy.loginfo(rospy.get_caller_id())

    if gs.state == GameState.INIT :
        rospy.loginfo("State = INIT")
    elif gs.state == GameState.WAIT_START :
        rospy.loginfo("State = WAIT_START")
    elif gs.state == GameState.RUNNING :
        rospy.loginfo("State = RUNNING")
    elif gs.state == GameState.PAUSED :
        rospy.loginfo("State = PAUSED")

    if gs.phase == GameState.PRE_GAME :
        rospy.loginfo("Phase = PRE_GAME")
    elif gs.phase == GameState.SETUP :
        rospy.loginfo("Phase = SETUP")
    elif gs.phase == GameState.EXPLORATION :
        rospy.loginfo("Phase = EXPLORATION")
    elif gs.phase == GameState.PRODUCTION :
        rospy.loginfo("Phase = PRODUCTION")
    elif gs.phase == GameState.POST_GAME :
        rospy.loginfo("Phase = POST_GAME")

    rospy.loginfo("Score = %d", gs.points)
    
    rospy.loginfo("temps = %d.%d", gs.game_time.secs, gs.game_time.nsecs);
    
def gs_listener():
    rospy.init_node('test_gameStateListener', anonymous=True)

    rospy.Subscriber("/refBoxComm/GameState", GameState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    gs_listener()



