#!/usr/bin/env python
import rospy
import tf
import roslib
import actionlib
import deplacement_msg.msg
from math import atan2, pi, cos, sin, sqrt
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

g_path = []
g_indexTraj = 0
g_trackingIsActive = False
g_pathFinished = False
g_stopRobot = True



class PID:
    def __init__(self, Kp, Ki, Kd, T):
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._err = 0
        self._I = 0
        self._T = T

    def update(self, err):
        self._I = self._I + err*self._T
        y = self._Kp * err + self._Ki * self._I + self._Kd * (err - self._err)/self._T
        self._err = err

        return y

def saturation(x, min, max):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x

class RateLimiter:
    def  __init__(self, upLimit, lowLimit, T):
        if lowLimit < upLimit:
            self._upLimit = upLimit
            self._lowLimit = lowLimit
        else:
            self._upLimit = lowLimit
            self._lowLimit = upLimit
        self._T = T
        self._lastX = 0



    def update(self, x):
        sigTemp = x - self._lastX
        sigTemp = saturation(sigTemp, self._lowLimit * self._T, self._upLimit * self._T)
        sigTemp = sigTemp + self._lastX
        self._lastX = sigTemp
        return sigTemp


g_anglePID = PID(1.5, 0, 0, 1/10.0)
g_speedPID = PID(0.1, 0, 0, 1/10.0)

g_speedLimiter = RateLimiter(-0.20, 0.2, 1/10.0)

#vitesse max robotino fixe
g_Vlim = 0.3
g_VminStatic = 0.05
g_Vmax = 0.3


# Publisher de consignes en vitesse
g_cmdVel_pub = rospy.Publisher('hardware/cmd_vel', Twist, queue_size=10)

def normalizeAngle(angle):
    while angle > pi:
        angle = angle - 2*pi
    while angle <= -pi:
        angle = angle + 2*pi
    return angle

class TrackPathAction(object):
  # create messages that are used to publish feedback/result
  _feedback = deplacement_msg.msg.TrackPathFeedback()
  _result   = deplacement_msg.msg.TrackPathResult()
  _state   = deplacement_msg.msg.TrackPathResult()

  def __init__(self, name):
    self._action_name = "navigation/trackPath"
    self._as = actionlib.SimpleActionServer(self._action_name, deplacement_msg.msg.TrackPathAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

  def execute_cb(self, goal):
    global g_path
    global g_stopRobot
    global g_pathFinished
    global g_indexTraj
    global g_trackingIsActive

    # helper variables
    r = rospy.Rate(2)
    success = False
    failure = False

    g_indexTraj = 0
    g_stopRobot = False
    g_pathFinished = False
    g_trackingIsActive = True


    if not failure:
        # Log
        rospy.loginfo('%s: Executing TrackPath' % (self._action_name))

        while not g_pathFinished:
            # rospy.loginfo('g_indexTraj : %d' % (g_indexTraj))
            # Fill the feedback
            self._feedback.percent_complete=g_indexTraj
            self._feedback.id=0

            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                g_stopRobot = True
                break

            # Publish the feedback
            self._as.publish_feedback(self._feedback)

            r.sleep()

        if g_pathFinished:
            success = True

    g_stopRobot = True
    g_trackingIsActive = False
    g_indexTraj = 0
    # Process the result if needed
    rospy.loginfo('Before Result')
    if success:
      rospy.loginfo('SUCCESS')
      self._result.result = self._result.FINISHED
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    elif failure:
      rospy.loginfo('FAILURE')
      self._result.result = self._result.ERROR
      rospy.loginfo('%s: Failed' % self._action_name)
      self._as.set_aborted(self._result)



def QuatMsg_to_quat(quat):
    q = []
    q.append(quat.x)
    q.append(quat.y)
    q.append(quat.z)
    q.append(quat.w)
    return q

def euler_from_quaternion_msg(orientation):
    q = QuatMsg_to_quat(orientation)
    euler = tf.transformations.euler_from_quaternion(q)
    return euler



def callbackOdom(data):
    global g_path
    global g_trackingIsActive
    global g_pathFinished
    global g_indexTraj
    global g_speedPID
    global g_anglePID
    global g_cmdVel_pub
    global g_Vlim
    global g_VminStatic
    global g_Vmax
    global g_speedLimiter

    cmdVel_msg = Twist()

    pose = Pose2D()
    (roll, pitch, yaw) = euler_from_quaternion_msg(data.pose.pose.orientation)
    pose.x = data.pose.pose.position.x
    pose.y = data.pose.pose.position.y
    pose.theta = yaw

    if g_stopRobot == True or len(g_path) == 0:
        g_cmdVel_pub.publish(cmdVel_msg)
        g_indexTraj = 0
        return

    # recherche du segment a suivre
    if g_indexTraj < len(g_path)-1 and g_trackingIsActive:
        # rospy.loginfo('Odom : g_indexTraj : %d' % (g_indexTraj))
        err = 0
        u = 10.0
        delta_x = 0
        delta_y = 0
        while u > 1.0 and g_indexTraj < len(g_path)-1:
            # rospy.loginfo('Odom While: g_indexTraj : %d' % (g_indexTraj))
            delta_x = g_path[g_indexTraj+1].pose.position.x - g_path[g_indexTraj].pose.position.x
            delta_y = g_path[g_indexTraj+1].pose.position.y - g_path[g_indexTraj].pose.position.y

            Rx = pose.x - g_path[g_indexTraj].pose.position.x
            Ry = pose.y - g_path[g_indexTraj].pose.position.y

            denom = (delta_x*delta_x + delta_y*delta_y)
            u = (Rx*delta_x + Ry*delta_y) / denom;
            # rospy.loginfo('Odom While: u : %f' % (u))
            if u>1.0:
                g_indexTraj = g_indexTraj+1

            err = (Ry*delta_x - Rx*delta_y) / denom
            #on a donc notre segment a suivre et l'erreur

        #calcul de l'orienation du segment
        segmentAngle = atan2(delta_y, delta_x)

        #calcul de l'erreur en angle
        errAngle = 0
        if (g_indexTraj >= len(g_path)-2):
            #sur le dernier segment on commence a reguler sur l'orientation finale
            (lastRoll, lastPitch, lastYaw) = euler_from_quaternion_msg(g_path[-1].pose.orientation)
            errAngle = normalizeAngle(lastYaw - pose.theta)
        else:
            errAngle = normalizeAngle(segmentAngle - pose.theta)

        cmdVel_msg.angular.z = g_anglePID.update(errAngle)
        cmdVel_msg.angular.z = saturation(cmdVel_msg.angular.z, -1.0, 1.0)

        #estimation de la vitesse max
        #parcours du chemin pour calcul amax et damax
        tmpIndex = g_indexTraj+1
        dWindow = .5
        dCurrent = 0
        while tmpIndex < len(g_path)-1 and dCurrent < dWindow:
            dx = g_path[tmpIndex+1].pose.position.x - g_path[tmpIndex].pose.position.x
            dy = g_path[tmpIndex+1].pose.position.y - g_path[tmpIndex].pose.position.y
            dSegment = sqrt(dx**2 + dy**2)
            dCurrent += dSegment
            tmpIndex+=1

        Vmin = g_VminStatic
        if (tmpIndex == len(g_path)-1):
            g_Vlim = g_speedLimiter.update((g_Vmax - Vmin)/dWindow * dCurrent + Vmin)
        else:
            g_Vlim = g_speedLimiter.update(g_Vmax)

        # rospy.loginfo('Vlim : %f' % (g_Vlim))

        #En repere segment local
        Vy = g_speedPID.update(-err)
        #saturer Vy a + ou -Vlim
        Vy = saturation(Vy, -g_Vlim, g_Vlim)

        Vx = sqrt(g_Vlim**2 - Vy**2)

        #Passer le vecteur (Vx Vy) en repere robot pour appliquer a cmdVel
        theta = segmentAngle - pose.theta
        cmdVel_msg.linear.x = Vx * cos(theta) - Vy * sin(theta)
        cmdVel_msg.linear.y = Vx * sin(theta) + Vy * cos(theta)


    elif g_indexTraj >= len(g_path)-1 and g_trackingIsActive:
        #arreter robot

        #orienation finale
        (lastRoll, lastPitch, lastYaw) = euler_from_quaternion_msg(g_path[-1].pose.orientation)
        err = normalizeAngle(lastYaw - pose.theta)
        if (abs(err) < 0.01):
            g_pathFinished = True
        else:
            cmdVel_msg.angular.z = g_anglePID.update(err)
            cmdVel_msg.angular.z = saturation(cmdVel_msg.angular.z, -1.0, 1.0)

            # rospy.loginfo('Vz = %f' % (cmdVel_msg.angular.z))

    g_cmdVel_pub.publish(cmdVel_msg)


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
