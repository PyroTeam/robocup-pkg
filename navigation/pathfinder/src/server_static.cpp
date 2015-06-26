#include "pathfinder/server_static_utils.h"

PathOrders pathReq = {-1,{0.0,0.0,0.0},{0.0,0.0},false};
pathfinder::AstarPath  pathFound;
pathfinder::AstarState pathfinderState;
int lastIdReceived = -1;

std::vector<GridPath> StaticTab;

int main(int argc, char **argv)
{
    defineOrigin(origin, 0.0, 0.56);

    initGridPath(StaticTab);
    initAstarPath(AstarTab, StaticTab);

    affichageTabPath(AstarTab);

    ros::init(argc, argv, "server_static");
    ros::NodeHandle n;

    pathfinderState.state = pathfinderState.LIBRE;
    pathfinderState.id = lastIdReceived;
    pathFound.id = lastIdReceived;

    ros::Publisher path_pub = n.advertise<pathfinder::AstarPath>("pathFound", 1000);
    ros::Publisher state_pub = n.advertise<pathfinder::AstarState>("pathfinderState", 1000);
    
    ros::ServiceServer service = n.advertiseService("generatePath", generatePath_callback);
    ROS_INFO("Ready to give the wanted path");

    boost::thread computeAStar_thread(&computeAStar_thread_function);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_pub.publish(pathFound);
        state_pub.publish(pathfinderState);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

bool generatePath_callback (pathfinder::GeneratePath::Request  &req,
                            pathfinder::GeneratePath::Response &res)
{
    ROS_INFO("GeneratePath request - ID : %d", req.id);

    if(!req.utilisePositionOdometry)
    {
        ROS_INFO("Pose Depart (Utilisation du parametre) : \nX: %f | Y : %f | Theta : %f",
            req.Depart.position.x,
            req.Depart.position.y,
            tf::getYaw(req.Depart.orientation));
    }
    else
    {
        ROS_INFO("Pose Depart (Utilisation de l'odometrie): \nX: %f | Y : %f | Theta : %f",
            0.0,
            0.0,
            0.0);
    }
    ROS_INFO("Pose Arrivee : \nX: %f | Y : %f | Theta : %f",
        req.Arrivee.position.x,
        req.Arrivee.position.y,
        tf::getYaw(req.Arrivee.orientation));

    if(pathReq.processing == true || req.id == lastIdReceived)
    {
        res.requeteAcceptee = false;
    }
    else
    {
        res.requeteAcceptee = true;
        lastIdReceived = req.id;

        pathReq.id              = req.id;
        pathReq.goalPose.x      = req.Arrivee.position.x;
        pathReq.goalPose.y      = req.Arrivee.position.y;
        pathReq.goalPose.yaw    = tf::getYaw(req.Arrivee.orientation);

        if(!req.utilisePositionOdometry)
        {
            pathReq.startPose.x = req.Depart.position.x;
            pathReq.startPose.y = req.Depart.position.y;
        }
        else
        {
            pathReq.startPose.x = 0;
            pathReq.startPose.y = 0;
        }
    } 

  return true;
}