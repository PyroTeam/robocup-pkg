
/* Classe abstraite qui permet d'aller vers une machine */

#include <string>

#include "Machine.h"
#include <common_utils/MPS.h>
#include "geometry_msgs/Pose2D.h"

Machine::Machine(int teamColor)
{
    m_name = (teamColor == CYAN)? "C-" : "M-";
}

Machine::~Machine(){}

std::string Machine::getType()
{
    return m_type;
}

geometry_msgs::Pose2D Machine::getEntryMachine()
{
    return m_entryMachine;
}

geometry_msgs::Pose2D Machine::getExitMachine()
{
    return m_exitMachine;
}

void Machine::majEntry(geometry_msgs::Pose2D point)
{
    m_entryMachine = point;
}

void Machine::majExit(geometry_msgs::Pose2D point)
{
    m_exitMachine = point;
}

/* Fonction abstraite qui permet d'aller vers une machine (Point centre/entree/sortie d'une machine) */

void Machine::goTo(geometry_msgs::Pose2D pt_dest)
{
    ROS_INFO("Going to point : x: %f; y: %f; theta: %f",pt_dest.x,pt_dest.y,pt_dest.theta);
    NavigationClientAction n_c;
    int stateOfNavigation = n_c.goToAPoint(pt_dest);
    if(stateOfNavigation == deplacement_msg::MoveToPoseResult::ERROR)
  {
    ROS_ERROR("Unable to go to requested point");
  }
    else
  {
    ROS_INFO ("Going to point - SUCCESS");
  }
}

/* Fonction qui permet de prendre un produit */
void Machine::grip()
{
    GripperClientSrv gsrv;
    gsrv.gripper_update(true);
}

/* Fonction qui permet de deposer un produit */
void Machine::let()
{
    GripperClientSrv gsrv;
    gsrv.gripper_update(false);
}

void Machine::startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
    FinalApproachingClient fa_c;
    fa_c.starting(machineType,machineSide,machineParameter);
}
