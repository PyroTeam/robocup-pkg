#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"

void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines);
void Create_Empty_Map(nav_msgs::OccupancyGrid &Map);
void Set_Wall(nav_msgs::OccupancyGrid &Map);
void Get_One_Point_Of_The_Rectangle(float x, float &xA, float y, float &yA, float theta, float largeur, float longueur);
void Set_Machines_In_Map(float rank, float theta, float xA, float yA, nav_msgs::OccupancyGrid &Map);