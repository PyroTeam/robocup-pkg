#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "Machine.h"

geometry_msgs::Pose2D LaserToRobot(geometry_msgs::Pose2D PosLaser);

geometry_msgs::Pose2D RobotToGlobal(geometry_msgs::Pose2D p, geometry_msgs::Pose2D odomRobot);

int getZone(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m);

int machineToArea(geometry_msgs::Pose2D m);

deplacement_msg::Landmarks convert(std::vector<Machine> mps);