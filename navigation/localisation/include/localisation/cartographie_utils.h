#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"

int getZone(geometry_msgs::Pose2D m);

geometry_msgs::Pose2D getCenter(int zone);

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m);

int machineToArea(geometry_msgs::Pose2D m);

deplacement_msg::Landmarks convert(std::vector<Machine> mps);