#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('path_finder')
import tf
import actionlib
import deplacement_msg.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

initialPose = Pose2D()
goalPose = Pose2D()

def QuatMsg_to_quat(quat):
    q = []
    q.append(quat.x)
    q.append(quat.y)
    q.append(quat.z)
    q.append(quat.w)
    return q

def InitPoseCallback(initPose):
    global initialPose
    initialPose.x = initPose.pose.pose.position.x
    initialPose.y = initPose.pose.pose.position.y

    q = QuatMsg_to_quat(initPose.pose.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    initialPose.theta = yaw

    rospy.loginfo("init Pose set to = %f,%f,%f", initialPose.x, initialPose.y, yaw)

def pathFeedback_cb(feedback):
    #rospy.loginfo("Elapsed Time : %f", feedback.processingTime.to_sec())
    return

def GoalPoseCallback(gPose):
    global goalPose
    global initialPose
    goalPose.x = gPose.pose.position.x
    goalPose.y = gPose.pose.position.y
    q = QuatMsg_to_quat(gPose.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    goalPose.theta = yaw

    rospy.loginfo("goal Pose set to = %f,%f,%f", goalPose.x, goalPose.y, yaw)

    client = actionlib.SimpleActionClient('navigation/generatePath', deplacement_msg.msg.GeneratePathAction)

    rospy.loginfo("Wait for Generate Path server")
    client.wait_for_server()

    rospy.loginfo("generate Goal")
    goal = deplacement_msg.msg.GeneratePathGoal(start=initialPose, goal=goalPose, timeout=rospy.Duration(secs=20, nsecs=0))

    rospy.loginfo("send goal")
    client.send_goal(goal, done_cb=None, active_cb=None, feedback_cb=pathFeedback_cb)

    rospy.loginfo("wait for result")
    client.wait_for_result()

    result = client.get_result()

    if (result.result ==  deplacement_msg.msg.GeneratePathResult.SUCCESS):
        rospy.loginfo("Path finder find a path")
        # attendre une seconde pour s'assurer que le pathTracker a recu le nouveau chemin
        rospy.sleep(1.0)
        # on execute le path_tracker
        clientPathTrack = actionlib.SimpleActionClient('navigation/trackPath', deplacement_msg.msg.TrackPathAction)

        rospy.loginfo("Wait for Path tracker server")
        clientPathTrack.wait_for_server()

        rospy.loginfo("generate Goal")
        goalPathTrack = deplacement_msg.msg.TrackPathGoal(id = 0)

        rospy.loginfo("send goal")
        clientPathTrack.send_goal(goalPathTrack, done_cb=None, active_cb=None, feedback_cb=None)

        rospy.loginfo("wait for result")
        clientPathTrack.wait_for_result()

        result = clientPathTrack.get_result()

        if (result.result ==  deplacement_msg.msg.TrackPathResult.FINISHED):
            rospy.loginfo("Path tracker finished")
        else:
            rospy.loginfo("Path tracker failed : %d", result.result)
    else:
        rospy.loginfo("Path not found : %d", result.result)

def callbackOdom(data):
    global initialPose
    initialPose.x = data.pose.pose.position.x
    initialPose.y = data.pose.pose.position.y


def PathRequest():
    rospy.init_node('pathRequest_node', anonymous=True)
    rospy.Subscriber("hardware/odom", Odometry, callbackOdom)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, InitPoseCallback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalPoseCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    PathRequest()
