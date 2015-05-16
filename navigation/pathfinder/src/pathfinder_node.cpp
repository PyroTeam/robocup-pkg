/**
 * \file pathfinder_node.cpp
 * \brief Fichier principal du noeud pathfinder.
 * \author Valentin Vergez
 * \version 1.0.1
 * \date 16 mai 2015
 *
 * Ce noeud propose un service 'generatePath' et retourne les résultats sur les topics
 *  - pathFound
 *  - path
 *  - pathfinderState
 *  
 * La recherche de chemin est effectué dans un Thread par une instance de la classe AStar.
 * La map est renseignée à AStar via le topic /grid souscrit sur une methode de callback de la classe.
 *
 */

/*==========  Includes  ==========*/
#include "pathfinder/pathfinder_node.hpp"

/*==========  Global variables  ==========*/
PathOrders g_pathReq = {-1,{0.0,0.0,0.0},{0.0,0},false};
pathfinder::AstarPath  g_pathFound;
pathfinder::AstarState g_pathfinderState;
int g_lastIdReceived = -1;
float g_current_x = 0.0;
float g_current_y = 0.0;
bool g_stop = false;

/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathfinder");
    ros::NodeHandle n;

    g_pathfinderState.state = g_pathfinderState.LIBRE;
    g_pathfinderState.id    = g_lastIdReceived;
    g_pathFound.id          = g_lastIdReceived;
    g_pathFound.path.header.frame_id = "map";

    ros::Publisher path_pub    = n.advertise<pathfinder::AstarPath>("pathFound", 1000);
    ros::Publisher path_simple_pub    = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher state_pub   = n.advertise<pathfinder::AstarState>("pathfinderState", 1000);
    
    ros::Subscriber sub_odo    = n.subscribe("/odom", 1000, odomCallback);

    ros::ServiceServer service = n.advertiseService("generatePath", generatePath_callback);
    
    ROS_DEBUG("Ready to compute A Star pathfinding.");

    boost::thread computeAStar_thread(&computeAStar_thread_function);

    ros::Rate loop_rate(10);
    while (ros::ok() && !g_stop)
    {
        g_pathFound.path.header.stamp = ros::Time::now();
        path_pub.publish(g_pathFound);
        path_simple_pub.publish(g_pathFound.path);    // Message standardsé lisible avec RVIZ
        state_pub.publish(g_pathfinderState);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

/*==========  Other functions  ==========*/
bool generatePath_callback( pathfinder::GeneratePath::Request  &req,
                            pathfinder::GeneratePath::Response &res)
{
    ROS_DEBUG("GeneratePath request - ID : %d", req.id);
    if(!req.utilisePositionOdometry)
    {
        ROS_DEBUG("Pose Depart (Utilisation du parametre) : \n\
                  X: %f |\
                  Y : %f",
                  req.Depart.position.x,
                  req.Depart.position.y);
    }
    else
    {
        ROS_DEBUG("Pose Depart (Utilisation de l'odometrie): \nX: %f | Y : %f | Theta : %f",0.0,0.0,0.0);
    }

    if(g_pathReq.processing == true || req.id == g_lastIdReceived)
    {
        res.requeteAcceptee = false;
    }
    else
    {
        res.requeteAcceptee = true;
        g_lastIdReceived = req.id;

        g_pathReq.id           = req.id;
        g_pathReq.goalPose.x   = req.Arrivee.position.x;
        g_pathReq.goalPose.y   = req.Arrivee.position.y;
        g_pathReq.goalPose.yaw = tf::getYaw(req.Arrivee.orientation);
        if(!req.utilisePositionOdometry)
        {
            g_pathReq.startPose.x = req.Depart.position.x;
            g_pathReq.startPose.y = req.Depart.position.y;
        }
        else
        {
            g_pathReq.startPose.x = g_current_x;
            g_pathReq.startPose.y = g_current_y;
        }
    }

  return true;
}

void odomCallback(nav_msgs::Odometry odom)
{
    g_current_x       = (odom).pose.pose.position.x;
    g_current_y       = (odom).pose.pose.position.y;
}

void computeAStar_thread_function()
{
    boost::posix_time::milliseconds     sleep_time(10);
    int lastId = -1;
    PathOrders actualOrders;

    AStar mapRobocup;
    Point *startPoint,*endPoint;
    std::vector<Point*> chemin;

    ros::NodeHandle n;
    ros::Subscriber sub_grid  = n.subscribe("/grid", 1000, &AStar::gridCallback, &mapRobocup);

    while(1 && !g_stop)
    {
        if(lastId != g_pathReq.id)
        {
            ROS_DEBUG("~~~~~~~~~~\ncomputeAStar : will start the path request id:%d\
                \nWith points start %f:%f and end %f:%f",g_pathReq.id,g_pathReq.startPose.x,g_pathReq.startPose.y,g_pathReq.goalPose.x,g_pathReq.goalPose.y);

            g_pathReq.processing = true;
            g_pathfinderState.state = g_pathfinderState.EN_COURS;

            lastId = g_pathReq.id;

            actualOrders = g_pathReq;

            if(mapRobocup.getNearestPoint(
                g_pathReq.startPose.x,
                g_pathReq.startPose.y,
                startPoint)){ROS_ERROR("getNearestPoint START Failed");}
            else{
                ROS_DEBUG("getNearestPoint START Suceeded, will start at %f:%f",startPoint->getX(),startPoint->getY());
            }
            if(mapRobocup.getNearestPoint(
                g_pathReq.goalPose.x,
                g_pathReq.goalPose.y,
                endPoint)){ROS_ERROR("getNearestPoint END Failed");}
            else{
                ROS_DEBUG("getNearestPoint END Suceeded, will stop at %f:%f",endPoint->getX(),endPoint->getY());
            }
            
            g_pathFound.id = actualOrders.id;
            g_pathFound.path.poses.erase(
                g_pathFound.path.poses.begin(),
                g_pathFound.path.poses.end());

            ros::Time startTime = ros::Time::now();

            mapRobocup.computeAStar(
                chemin,
                startPoint,
                endPoint);

            long long int timeElapsed = ros::Time::now().toNSec() - startTime.toNSec();

            ROS_INFO("------\ncomputeAStar Ended. Time elapsed = %lld / %lldms", timeElapsed, timeElapsed/1000000);

            g_pathfinderState.id = actualOrders.id;

            if( chemin.size() != 0 &&
                chemin.front() == startPoint &&
                chemin.back() == endPoint)
            {
                ROS_INFO("Path finding sucessfull. Will publish path soon.");
                std::size_t i;

                for(i=0;i<chemin.size()-1;++i)
                {
                    geometry_msgs::PoseStamped point;
                    point.pose.position.x = chemin[i]->getX();
                    point.pose.position.y = chemin[i]->getY();
                    g_pathFound.path.poses.push_back(point);
                }
                geometry_msgs::PoseStamped pointFinal;
                pointFinal.pose.position.x = chemin[i]->getX();
                pointFinal.pose.position.y = chemin[i]->getY();
                // pointFinal.pose.orientation =
                //     tf::createQuaternionMsgFromYaw(actualOrders.goalPose.yaw);
                // Les quatre lignes qui suivent viennent masquer un bug -> la fonction précédente retourne des NaN
                pointFinal.pose.orientation.x = 0;
                pointFinal.pose.orientation.y = 0;
                pointFinal.pose.orientation.z = 0;
                pointFinal.pose.orientation.w = 0;
                g_pathFound.path.poses.push_back(pointFinal);

                g_pathfinderState.state = g_pathfinderState.SUCCES;
            }
            else
            {
                g_pathfinderState.state = g_pathfinderState.ECHEC;
            }

            g_pathReq.processing = false;
        }

        boost::this_thread::sleep(sleep_time);
    }
}
