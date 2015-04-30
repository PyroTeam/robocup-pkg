#!/usr/bin/env python
import rospy
import tf
import actionlib
import deplacement_msg.msg
from math import atan2, pi, cos, sin
from geometry_msgs.msg import Twist, PoseStamped, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from pathfinder.msg import AstarPath

# Publisher de consignes en vitesse
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Publisher de debug rqt_plot
pubAngle = rospy.Publisher('/t_angle', Float32, queue_size=10)
pubErrAngle = rospy.Publisher('/t_err_angle', Float32, queue_size=10)
pubInfos = rospy.Publisher('/tracker_info', Float32, queue_size=10)

# Tableau de points a suivre et dernier chemin sauve (PoseStamped)
points = []
path = []
pathId = 0
pointsId = 0
stopRobot = True
pathFinished = False
cptCmdVel = 5


#*====================================
#            CLASS ACTION            =
#===================================*/

class TrackPathAction(object):
  # create messages that are used to publish feedback/result
  _feedback = deplacement_msg.msg.TrackPathFeedback()
  _result   = deplacement_msg.msg.TrackPathResult()
  _state   = deplacement_msg.msg.TrackPathResult()

  def __init__(self, name):
    self._action_name = "trackPath"
    self._as = actionlib.SimpleActionServer(self._action_name, deplacement_msg.msg.TrackPathAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    global points
    global path
    global pathId
    global pointsId
    global stopRobot
    global pathFinished

    # helper variables
    r = rospy.Rate(1)
    success = False
    failure = False

    ## Check l'id du Path
    idPath = goal.id
    # Already running the good path
    if pointsId == idPath:
        pass
    # New path requested, path received from pathfinder 
    elif pathId == idPath:
        points = path
        pointsId = pathId
        stopRobot = False
        pathFinished = False
    # New path requested, but path not received from pathfinder
    else:
        failure = True
        stopRobot = True

    if not failure:
        # Log
        rospy.loginfo('%s: Executing, creating TrackPath sequence of order, %i' % (self._action_name, goal.id))

        # Do we have finished
        while not pathFinished:

            # Fill the feedback
            self._feedback.percent_complete=50
            self._feedback.id=idPath
            
            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                stopRobot = True
                break

            # Publish the feedback
            self._as.publish_feedback(self._feedback)

            rospy.sleep(0.1)

        if pathFinished:
            success = True        
      
    # Process the result if needed
    rospy.loginfo('Before Result')
    if success:
      rospy.loginfo('SUCESS')
      self._result.result = self._result.FINISHED
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
    elif failure:
      rospy.loginfo('FAILURE')
      self._result.result = self._result.ERROR
      rospy.loginfo('%s: Failed' % self._action_name)
      self._as.set_aborted(self._result)

#*-----  End of CLASS ACTION  ------*/


















def closestPoint(segmentStart, segmentStop, point):
    closestPoint = Point()

    xDelta = segmentStop.x - segmentStart.x
    yDelta = segmentStop.y - segmentStart.y

    if ((xDelta == 0) and (yDelta == 0)):
        closestPoint = segmentStart
        return closestPoint

    u = ((point.x - segmentStart.x) * xDelta + (point.y - segmentStart.y) * yDelta) \
    /   (xDelta * xDelta + yDelta * yDelta)

    if (u < 0):
        closestPoint = segmentStart
    elif (u > 1):
        closestPoint = segmentStop
    else:
        closestPoint.x = segmentStart.x + u*xDelta
        closestPoint.y = segmentStart.y + u*yDelta

    return closestPoint;


def callbackOdom(data):
    global points
    global path
    global pathId
    global pointsId
    global stopRobot
    global pathFinished

    # Chercher point le plus proche
    pose = Point()
    pose.x = data.pose.pose.position.x
    pose.y = data.pose.pose.position.y
    closest = Point()
    display = False
    if len(points) != 0:
        stopRobot = False
    pointObjectif = points[0].pose.position
    for i in xrange(len(points)):
        display = True
        sizePath = len(points)
        rospy.loginfo(sizePath)
        if sizePath >= 1:
            start = points[0].pose.position
            if sizePath >= 2:
                stop = points[1].pose.position
                pointObjectif = stop
                closest = closestPoint(start, stop, pose)
                if closest == stop:
                    points.pop(0)
                    continue
                else:
                    break
            else:
                closest = start
                stopRobot = True
                pathFinished = True

    if display:
        rospy.loginfo(closest)
        display = False
    
    # Determiner la distance d'avance
    distAvance = 0.1

    # Determiner point d'avance
    pointAvance = Point()

    pointSuiv = Point()
    if len(points) >=2 :
        pointSuiv = points[1].pose.position
    else :
        pointSuiv = points[0].pose.position

    dX = pointSuiv.x-closest.x
    dY = pointSuiv.y-closest.y
    ang = atan2(dY, dX)

    pointAvance.x = closest.x + distAvance*cos(ang)
    pointAvance.y = closest.y + distAvance*sin(ang)

    # Rejoindre point d'avance
    ## Viser
    adj = pointAvance.x - data.pose.pose.position.x
    opp = pointAvance.y - data.pose.pose.position.y
    angle = atan2(opp, adj)

    quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    errAngle = (angle - yaw)
    errAngle = ((errAngle+pi) % (2*pi)) - pi

    errAnglePointSuiv = (ang - yaw)
    errAnglePointSuiv = ((errAnglePointSuiv+pi) % (2*pi)) - pi

    vitAngle = errAnglePointSuiv*1

    rospy.loginfo("----------------")
    rospy.loginfo(stopRobot)
    rospy.loginfo("PointObjectif : ")
    rospy.loginfo(pointObjectif)
    rospy.loginfo("Pose : ")
    rospy.loginfo(data.pose.pose.position)
    rospy.loginfo(yaw)
    rospy.loginfo("PointClosest : ")
    rospy.loginfo(closest)
    rospy.loginfo("PointAvance : ")
    rospy.loginfo(pointAvance)
    rospy.loginfo("Angle %f - VitAngle %f",angle,vitAngle)
        
    if vitAngle > 30:
        vitAngle = 30
    elif vitAngle < -30:
        vitAngle = -30

    # Publication du message sur le topic
    vel_msg = Twist()
    # stopRobot = True

    global cptCmdVel
    if not stopRobot:
        vel_msg.linear.x = 0.2
        vel_msg.linear.y = errAngle/10
        vel_msg.angular.z = vitAngle*1
        cptCmdVel = 5
    else:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0

        cptCmdVel-=1


    if not stopRobot or (stopRobot and cptCmdVel > 0):
        pub.publish(vel_msg)
    
    pubInfos.publish(ang)
    pubAngle.publish(angle)
    pubErrAngle.publish(errAngle)


def callbackPath(data):
    global points
    global path
    global pathId
    global pointsId
    global stopRobot
    global pathFinished

    path = data.path.poses
    pathId = data.id

def path_tracker_node():
    rospy.init_node('path_tracker_node', anonymous=False)
    rospy.Subscriber("/odom", Odometry, callbackOdom)
    rospy.Subscriber("/pathFound", AstarPath, callbackPath)

    TrackPathAction(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    try:
        path_tracker_node()
    except rospy.ROSInterruptException:
        pass
