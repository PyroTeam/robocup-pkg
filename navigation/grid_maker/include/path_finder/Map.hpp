#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"

void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines);
void Create_Empty_Map(nav_msgs::OccupancyGrid &Map);
void Set_Wall(nav_msgs::OccupancyGrid &Map);
void Set_Forbidden_Zone(nav_msgs::OccupancyGrid &Map);


int getCell(nav_msgs::OccupancyGrid Map, float x, float y);
int drawRect(nav_msgs::OccupancyGrid &Map, float x, float y, float theta, float height, float width, float margin);
int getZone(float x, float y);
int eraseZone(nav_msgs::OccupancyGrid &Map, int zone);