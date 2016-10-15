#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "manager_msg/activity.h"
#include "manager_msg/order.h"

#include "GtServerSrv.h"
#include "LocaSubscriber.h"
#include "FeuClientAction.h"
#include "ReportingMachineSrvClient.h"
#include "Machine.h"

#include "comm_msg/GameState.h"
#include "generateur_taches/gameState.h"
#include "common_utils/types.h"

using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc,argv,"report_all_machines");
    ros::NodeHandle n; // création du noeud
    int nb_robot;
    n.param<int>("robotNumber",nb_robot,0);  // nb robot, par défaut 0

    ros::Rate loop_rate(1);

    std::string teamColorStr;
    n.param<std::string>("teamColor", teamColorStr, "cyan");
    int teamColor = (teamColorStr == "magenta")? MAGENTA: CYAN;

    GameState gameState;
    LocaSubscriber locaSubscriber;
    ReportingMachineSrvClient reportClient;
    MyElements m_elements(teamColor);

    ROS_INFO("Waiting exploration phase to begin");
    do {
        ROS_INFO_STREAM("Phase" << int(gameState.getPhase()));
        ros::spinOnce();
        loop_rate.sleep();
    } while (gameState.getPhase() != comm_msg::GameState::EXPLORATION && ros::ok());

    ROS_INFO("Exploration phase started");

    do {
        ROS_INFO_STREAM("Temps de jeu : " << gameState.getTime());
        ros::spinOnce();
        loop_rate.sleep();
    } while (gameState.getTime() < ros::Time(4*60-10) && ros::ok());//4*60-10

    ROS_INFO("Time is running out, report all machines");

    Machine *machine = nullptr;
    for(auto &it : locaSubscriber.machines())
    {
        if (it.orientationOk && common_utils::isMyTeam(it.idIn, teamColor))
        {
            machine = m_elements.getMachineFromTag(it.idIn);
            reportClient.reporting(machine->getName(), "", it.zone);
            ROS_INFO("Report machine: zone %d, name: %s", it.zone, machine->getName());
        }
    }

    return 0;
}
