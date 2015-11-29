#ifndef _SERVER_HEADER_
#define _SERVER_HEADER_

/*==========  Includes  ==========*/
//StdLib
    #include <iostream>

    // Perso
    #include "pathfinder/AStar.hpp"
    #include "pathfinder/Point.hpp"

    #include "pathfinder/GeneratePath.h"
    #include "pathfinder/AstarPath.h"
    #include "pathfinder/AstarState.h"

    #include "ros/ros.h"
    #include "tf/transform_datatypes.h"
    #include "nav_msgs/Path.h"
    #include "nav_msgs/Odometry.h"
    #include "nav_msgs/OccupancyGrid.h"

    #include <boost/thread/thread.hpp>


/*==========  Types  ==========*/
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

/*==========  Declarations  ==========*/
void odomCallback(nav_msgs::Odometry odom);

void gridCallback(nav_msgs::OccupancyGridConstPtr grid);

bool generatePath_callback( pathfinder::GeneratePath::Request  &req,
                            pathfinder::GeneratePath::Response &res);

void computeAStar_thread_function();

#endif // _SERVER_HEADER_
