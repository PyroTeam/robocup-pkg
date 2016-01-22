#!/usr/bin/env python
import rospy
import roslib
import actionlib
from math import atan2, pi, cos, sin, sqrt
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

g_path = []
g_indexTraj = 0
g_trackingIsActive = False
g_pathFinished = False

# Publisher de consignes en vitesse
cmdVel_pub = rospy.Publisher('hardware/cmd_vel', Twist, queue_size=10)

def QuatMsg_to_quat(quat):
    q = []
    q.append(quat.x)
    q.append(quat.y)
    q.append(quat.z)
    q.append(quat.w)
    return q

def callbackOdom(data):
    global g_path
    global g_trackingIsActive
    global g_pathFinished
    cmdVel_msg = Twist()

    pose = Pose2D()
    pose.x = data.pose.pose.position.x
    pose.y = data.pose.pose.position.y
    q = QuatMsg_to_quat(data.pose.pose.orientation)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    pose.theta = yaw

    # recherche du segment à suivre
    if g_indexTraj < len(g_path) and g_trackingIsActive:
        err = 0
        u = 10.0
        while u > 1.0:
            delta_x = g_path[g_indexTraj+1].pose.position.x - g_path[g_indexTraj].pose.position.x
            delta_y = g_path[g_indexTraj+1].pose.position.y - g_path[g_indexTraj].pose.position.y

            Rx = pose.x - g_path[g_indexTraj].pose.position.x
            Rx = pose.y - g_path[g_indexTraj].pose.position.y

            denom = (delta_x*delta_x + delta_y*delta_y)
            u = (Rx*delta_x + Ry*delta_y) / denom;

            if u>1.0:
                g_indexTraj = g_indexTraj+1

            err = (Ry*delta_x - Rx*delta_y) / denom
            #on a donc notre segment à suivre et l'erreur

        #calcul de l'orienation du segment
        #calcul de l'erreur en angle
        #puis cmdVel_msg.angular.z = PID(a)

        #En repère segment local
        #Vy = PID(err)
        #saturer Vy à + ou -Vlim
        #Vx = sqrt( Vlim**2 - Vy**2)
        #Passer le vecteur (Vx Vy) en repère robot pour appliquer à cmdVel
        cmdVel_pub.publish(cmdVel_msg)

    elif g_indexTraj >= len(g_path) and g_trackingIsActive:
        g_pathFinished = True
        #arreter robot
        cmdVel_pub.publish(cmdVel_msg)
        #orienation finale



def callbackPath(data):
    global g_path
    g_path = data.poses


def path_tracker_node():
    rospy.init_node('path_tracker_node', anonymous=False)
    rospy.Subscriber("hardware/odom", Odometry, callbackOdom)
    rospy.Subscriber("navigation/pathSmooth", Path, callbackPath)

    TrackPathAction(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    try:
        path_tracker_node()
    except rospy.ROSInterruptException:
        pass
