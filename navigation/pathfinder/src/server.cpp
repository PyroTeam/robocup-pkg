/*==========  Includes  ==========*/
#include "pathfinder/server.hpp"

/*==========  Global variables  ==========*/
PathOrders pathReq = {-1,{0.0,0.0,0.0},{0.0,0},false};
pathfinder::AstarPath  pathFound;
pathfinder::AstarState pathfinderState;
int lastIdReceived = -1;
float current_x = 0.0;
float current_y = 0.0;
bool stop = false;

/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    ros::NodeHandle n;

    pathfinderState.state = pathfinderState.LIBRE;
    pathfinderState.id    = lastIdReceived;
    pathFound.id          = lastIdReceived;
    pathFound.path.header.frame_id = "map";

    ros::Publisher path_pub    = n.advertise<pathfinder::AstarPath>("pathFound", 1000);
    ros::Publisher path_simple_pub    = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher state_pub   = n.advertise<pathfinder::AstarState>("pathfinderState", 1000);
    
    ros::Subscriber sub_odo    = n.subscribe("/odom", 1000, odomCallback);

    ros::ServiceServer service = n.advertiseService("generatePath", generatePath_callback);
    
    ROS_INFO("Ready to compute A Star pathfinding.");

    boost::thread computeAStar_thread(&computeAStar_thread_function);

    ros::Rate loop_rate(10);
    while (ros::ok() && !stop)
    {
        pathFound.path.header.stamp = ros::Time::now();
        path_pub.publish(pathFound);
        path_simple_pub.publish(pathFound.path);    // Message standardsé lisible avec RVIZ
        state_pub.publish(pathfinderState);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

/*==========  Other functions  ==========*/
bool generatePath_callback( pathfinder::GeneratePath::Request  &req,
                            pathfinder::GeneratePath::Response &res)
{
    ROS_INFO("GeneratePath request - ID : %d", req.id);
    if(!req.utilisePositionOdometry)
    {
        ROS_INFO("Pose Depart (Utilisation du parametre) : \n\
                  X: %f |\
                  Y : %f |\
                  Theta : %f",
                  req.Depart.position.x,
                  req.Depart.position.y,
                  tf::getYaw(req.Depart.orientation));
    }
    else
    {
        ROS_INFO("Pose Depart (Utilisation de l'odometrie): \nX: %f | Y : %f | Theta : %f",0.0,0.0,0.0);
    }
    ROS_INFO("Pose Arrivee :\n\
              X: %f |\
              Y : %f |\
              Theta : %f",
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

        pathReq.id           = req.id;
        pathReq.goalPose.x   = req.Arrivee.position.x;
        pathReq.goalPose.y   = req.Arrivee.position.y;
        pathReq.goalPose.yaw = tf::getYaw(req.Arrivee.orientation);
        if(!req.utilisePositionOdometry)
        {
            pathReq.startPose.x = req.Depart.position.x;
            pathReq.startPose.y = req.Depart.position.y;
        }
        else
        {
            pathReq.startPose.x = current_x;
            pathReq.startPose.y = current_y;
        }
    }

  return true;
}

void odomCallback(nav_msgs::Odometry odom)
{
    current_x       = (odom).pose.pose.position.x;
    current_y       = (odom).pose.pose.position.y;
}

void computeAStar_thread_function()
{
    boost::posix_time::milliseconds     sleep_time(10);
    int lastId = -1;
    PathOrders actualOrders;

    Map mapRobocup;
    Point *startPoint,*endPoint;
    std::vector<Point*> chemin;

    ros::NodeHandle n;
    ros::Subscriber sub_grid  = n.subscribe("/grid", 1000, &Map::gridCallback, &mapRobocup);

    while(1 && !stop)
    {
        if(lastId != pathReq.id)
        {
            ROS_INFO("~~~~~~~~~~\ncomputeAStar : will start the path request id:%d\
                \nWith points start %f:%f and end %f:%f",pathReq.id,pathReq.startPose.x,pathReq.startPose.y,pathReq.goalPose.x,pathReq.goalPose.y);

            pathReq.processing = true;
            pathfinderState.state = pathfinderState.EN_COURS;

            lastId = pathReq.id;

            actualOrders = pathReq;

            if(mapRobocup.getNearestPoint(
                pathReq.startPose.x,
                pathReq.startPose.y,
                startPoint)){ROS_ERROR("getNearestPoint START Failed");}
            else{
                ROS_INFO("getNearestPoint START Suceeded, will start at %f:%f",startPoint->getX(),startPoint->getY());
            }
            if(mapRobocup.getNearestPoint(
                pathReq.goalPose.x,
                pathReq.goalPose.y,
                endPoint)){ROS_ERROR("getNearestPoint END Failed");}
            else{
                ROS_INFO("getNearestPoint END Suceeded, will stop at %f:%f",endPoint->getX(),endPoint->getY());
            }
            
            pathFound.id = actualOrders.id;
            pathFound.path.poses.erase(
                pathFound.path.poses.begin(),
                pathFound.path.poses.end());

            ros::Time startTime = ros::Time::now();

            mapRobocup.computeAStar(
                chemin,
                startPoint,
                endPoint);

            long long int timeElapsed = ros::Time::now().toNSec() - startTime.toNSec();

            ROS_INFO("------\ncomputeAStar Ended. Time elapsed = %lld", timeElapsed);

            pathfinderState.id = actualOrders.id;

            if( chemin.size() != 0 &&
                chemin.front() == startPoint &&
                chemin.back() == endPoint)
            {
                ROS_WARN("Path finding sucessfull. Will publish path soon.");
                std::size_t i;

                for(i=0;i<chemin.size()-1;++i)
                {
                    geometry_msgs::PoseStamped point;
                    point.pose.position.x = chemin[i]->getX();
                    point.pose.position.y = chemin[i]->getY();
                    pathFound.path.poses.push_back(point);
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
                pathFound.path.poses.push_back(pointFinal);

                pathfinderState.state = pathfinderState.SUCCES;
            }
            else
            {
                ROS_WARN("Path finding unsucessfull ...");
                pathfinderState.state = pathfinderState.ECHEC;
            }

            pathReq.processing = false;
        }

        boost::this_thread::sleep(sleep_time);
    }
}
