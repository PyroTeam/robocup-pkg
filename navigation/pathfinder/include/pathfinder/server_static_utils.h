#ifndef _SERVER_STATIC_UTILS_HEADER_
#define _SERVER_STATIC_UTILS_HEADER_

#include <iostream>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "pathfinder/GeneratePath.h"
#include "pathfinder/AstarPath.h"
#include "pathfinder/AstarState.h"

#include <boost/thread/thread.hpp>
#include <cmath>
#include <vector>

#define OFFSET_GRID 0.56 
#define POSE_TOLERANCE  0.20

typedef struct{
    int id;
    struct
    {
        float x;
        float y;      
        float yaw;  
    } goalPose;
    struct
    {
        float x;
        float y;        
    } startPose;
    char processing;
} PathOrders;

typedef struct GridPoint
{
    GridPoint(int a, int b)
    {
        x = a;
        y = b;
    }
    
    int x;
    int y;
} GridPoint;

typedef std::vector<GridPoint> GridPath;

typedef std::vector<GridPath> tabGridPath;

extern geometry_msgs::PoseStamped origin;
extern std::vector<pathfinder::AstarPath> AstarTab;
extern PathOrders pathReq;
extern pathfinder::AstarPath  pathFound;
extern pathfinder::AstarState pathfinderState;
extern int lastIdReceived;
extern std::vector<GridPath> StaticTab;

geometry_msgs::Pose quat_normalize(const geometry_msgs::Pose &p);

void defineOrigin(geometry_msgs::PoseStamped &origin, float xorigin, float yorigin);

geometry_msgs::PoseStamped calculPoint (int xgrille,
                                        int ygrille,
                                        geometry_msgs::PoseStamped origin,
                                        float OffsetGrid);

void affichageTabPath(std::vector<pathfinder::AstarPath> &AstarTab);

void affichePath(pathfinder::AstarPath &AStar);

void inverserChemin(GridPath &Chemin);

void initGridPath(std::vector<GridPath> &tab);

void initAstarPath (std::vector<pathfinder::AstarPath> &AstarTab,
                    tabGridPath &StaticTab);

void getPathFromStartPoint (pathfinder::AstarPath tab,
                            float xdepart, 
                            float ydepart,
                            std::vector<pathfinder::AstarPath> &tabResult);

void getPathFromEndPoint   (std::vector<pathfinder::AstarPath> tabResult,
                            float xarrivee,
                            float yarrivee,
                            pathfinder::AstarPath &Path);

bool generatePath_callback (pathfinder::GeneratePath::Request  &req,
         					pathfinder::GeneratePath::Response &res);

void computeAStar_thread_function();

#endif // _SERVER_STATIC_UTILS_HEADER_