#include "ros/ros.h"
#include "path_tracker/trackPathAction.h"
#include "path_tracker/dataMapObstacle.h"

/*nav_msgs::OccupancyGrid g_grid;
bool g_receiveGrid = false;

void gridCallback(const nav_msgs::OccupancyGrid &grid)
{
    g_grid = grid;
    g_receiveGrid = true;
}*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PathTracker");
    ros::NodeHandle nh;
    //ros::Subscriber grid_sub = nh.subscribe("/gridObstacles", 1000, &gridCallback);

    ros::spinOnce();
    TrackPathAction pathTrack("/trackPath");
    //DataMapObstacle map;

    /*geometry_msgs::Pose pose;
    pose.position.x = -2;
    pose.position.y = 1;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    geometry_msgs::Point point;
    point.x = 3;
    point.y = 1;
*/
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        /*if (g_receiveGrid)
        {
            map.calculObstacle(pose, point, map.calculDistance(pose.position, point));
        }*/
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
