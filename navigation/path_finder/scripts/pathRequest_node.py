#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('path_finder')
import tf
from path_finder.srv import GeneratePath
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

initialPose = Pose()
goalPose = Pose()

def QuatMsg_to_quat(quat):
    q = []
    q.append(quat.x)
    q.append(quat.y)
    q.append(quat.z)
    q.append(quat.w)
    return q

def InitPoseCallback(initPose):
    global initialPose
    initialPose = initPose.pose.pose

    q = QuatMsg_to_quat(initPose.pose.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    rospy.loginfo("init Pose set to = %f,%f,%f", initialPose.position.x, initialPose.position.y, yaw);

def GoalPoseCallback(gPose):
    global goalPose
    global initialPose
    goalPose = gPose.pose
    q = QuatMsg_to_quat(gPose.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    rospy.loginfo("goal Pose set to = %f,%f,%f", goalPose.position.x, goalPose.position.y, yaw);

    rospy.wait_for_service('generatePath')
    try:
        generatePath = rospy.ServiceProxy('generatePath', GeneratePath)
        resp1 = generatePath(0, goalPose, initialPose, False)
        print resp1.requeteAcceptee
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def PathRequest():
    rospy.init_node('pathRequest_node', anonymous=True)

    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, InitPoseCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalPoseCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    PathRequest()
