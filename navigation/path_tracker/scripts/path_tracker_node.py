#!/usr/bin/env python
import rospy
from math import atan2, pi, cos, sin
from geometry_msgs.msg import Twist, PoseStamped, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Path

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
points = []

# point = PoseStamped()
# point.pose.position.x = 1
# point.pose.position.y = 10
# points.append(point)
# point = PoseStamped()
# point.pose.position.x = 1
# point.pose.position.y = 1
# points.append(point)
# point = PoseStamped()
# point.pose.position.x = 1
# point.pose.position.y = 2
# points.append(point)
# point = PoseStamped()
# point.pose.position.x = 2
# point.pose.position.y = 2
# points.append(point)
# point = PoseStamped()
# point.pose.position.x = 4
# point.pose.position.y = 4
# points.append(point)
# point = PoseStamped()
# point.pose.position.x = 9
# point.pose.position.y = 4
# points.append(point)

coeffX= 1.3
coeffY=1.3
offsetY=-3
# P*coeffY
point = PoseStamped()
point.pose.position.x = 0*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1.5*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1.75*coeffX
point.pose.position.y = 9.75*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1.87*coeffX
point.pose.position.y = 9.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1.75*coeffX
point.pose.position.y = 9.25*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 1*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)

# Y
point = PoseStamped()
point.pose.position.x = 2.5*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 3.5*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 3*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 2.5*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 3*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 2.5*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)

# R
point = PoseStamped()
point.pose.position.x = 4*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.5*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.75*coeffX
point.pose.position.y = 9.75*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.87*coeffX
point.pose.position.y = 9.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.75*coeffX
point.pose.position.y = 9.25*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 4.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 5*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)

# O
point = PoseStamped()
point.pose.position.x = 6.5*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 6*coeffX
point.pose.position.y = 8.25*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 5.75*coeffX
point.pose.position.y = 8.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 5.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 5.75*coeffX
point.pose.position.y = 9.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 6*coeffX
point.pose.position.y = 9.75*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 6.5*coeffX
point.pose.position.y = 10*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 7*coeffX
point.pose.position.y = 9.75*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 7.25*coeffX
point.pose.position.y = 9.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 7.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 7.25*coeffX
point.pose.position.y = 8.5*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 7*coeffX
point.pose.position.y = 8.25*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 6.5*coeffX
point.pose.position.y = 8*coeffY+offsetY
points.append(point)
point = PoseStamped()
point.pose.position.x = 6.5*coeffX
point.pose.position.y = 9*coeffY+offsetY
points.append(point)

point = points[0].pose.position

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


def callback(data):

    # Chercher point le plus proche
    pose = Point()
    pose.x = data.x
    pose.y = data.y
    closest = Point()
    display = False
    stopTurtle = False
    for i in xrange(len(points)):
        display = True
        sizePath = len(points)
        rospy.loginfo(sizePath)
        if sizePath >= 1:
            start = points[0].pose.position
            if sizePath >= 2:
                stop = points[1].pose.position
                closest = closestPoint(start, stop, pose)
                if closest == stop:
                    points.pop(0)
                    continue
                else:
                    break
            else:
                closest = start
                stopTurtle = True

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
    adj = pointAvance.x - data.x
    opp = pointAvance.y - data.y
    angle = atan2(opp, adj)

    errAngle = (angle - data.theta)
    errAngle = ((errAngle+pi) % (2*pi)) - pi
    vitAngle = errAngle*30
    rospy.loginfo("Angle %f",angle)

    # Publication du message sur le topic
    vel_msg = Twist()
    if not stopTurtle:
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.angular.z = vitAngle
    else :
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0

    pub.publish(vel_msg)

def path_tracker_node():
    rospy.init_node('path_tracker_node', anonymous=False)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        path_tracker_node()
    except rospy.ROSInterruptException:
        pass