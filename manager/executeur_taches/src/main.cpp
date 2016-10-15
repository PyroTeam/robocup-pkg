#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "manager_msg/activity.h"
#include "manager_msg/order.h"

#include "GtServerSrv.h"
#include "LocaSubscriber.h"
#include "FeuClientAction.h"
#include "Machine.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"executeur_taches");
    ros::NodeHandle n; // création du noeud
    int nb_robot;
    n.param<int>("robotNumber",nb_robot,0);  // nb robot, par défaut 0
    ros::Rate loop_rate(1);

    std::string teamColorStr;
    n.param<std::string>("teamColor", teamColorStr, "cyan");
    int teamColor = (teamColorStr == "magenta")? MAGENTA: CYAN;

    /* service reponse au générateur de taches */
    GtServerSrv gtsrv(teamColor);
    gtsrv.setId(1);
    ros::ServiceServer service = n.advertiseService("manager/order", &GtServerSrv::responseToGT, &gtsrv);

    /* Publisher topic générateur de taches */
    ros::Publisher activite_pub = n.advertise<manager_msg::activity>("manager/task_exec_state", 1000);

    while(ros::ok())
    {
        //ROS_INFO("I'm READY ! ");
        activite_pub.publish(gtsrv.getActivityMsg());

        /* Let's Spin until the end of the world !! */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
