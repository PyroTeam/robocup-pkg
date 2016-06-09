#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('navigation_manager')
import tf
import actionlib
from deplacement_msg.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

goalPose = Pose2D()

def QuatMsg_to_quat(quat):
    q = []
    q.append(quat.x)
    q.append(quat.y)
    q.append(quat.z)
    q.append(quat.w)
    return q



def move_to_pose_client():

    global g_client;


def GoalMoveCallback(gMove):
    g_client.cancel_all_goals()

    q = QuatMsg_to_quat(gMove.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)

    global goalPose
    goalPose.x = gMove.pose.position.x
    goalPose.y = gMove.pose.position.y
    goalPose.theta = yaw
    rospy.loginfo("goal Pose set to = %f,%f,%f", goalPose.x, goalPose.y, goalPose.theta)

    # Creates a goal to send to the action server.
    goal = MoveToPoseGoal(goalPose)

    # Sends the goal to the action server.
    g_client.send_goal(goal)

    # # Waits for the server to finish performing the action.
    g_client.wait_for_result(rospy.Duration.from_sec(60.1))

    # # Prints out the result of executing the action
    #rospy.loginfo("Result : %f", g_client.get_result())

def MoveRequest():
    rospy.init_node('moveRequest_node', anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalMoveCallback)

    global g_client
    # Creates the SimpleActionClient, passing the type of the action
    g_client = actionlib.SimpleActionClient('navigation/moveToPose', MoveToPoseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    g_client.wait_for_server()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    MoveRequest()
