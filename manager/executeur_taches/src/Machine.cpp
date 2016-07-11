
/* Classe abstraite qui permet d'aller vers une machine */

#include <string>

#include "Machine.h"
#include "geometry_msgs/Pose2D.h"

/* Constructeur */
Machine::Machine(int teamColor)
{
	m_name = (teamColor == CYAN)? "C-" : "M-";
	m_centerMachine.x = 0.0;
	m_centerMachine.y = 0.0;
	m_centerMachine.theta = 0.0;
	m_entryMachine.x = 0.0;
	m_entryMachine.y = 0.0;
	m_entryMachine.theta = 0.0;
	m_exitMachine.x = 0.0;
	m_exitMachine.y = 0.0;
  m_exitMachine.theta = 0.0;

  m_orientationOk = false;
}

/* Destructeur */
Machine::~Machine(){}

/* Méthodes */
std::string Machine::getType()
{
	return m_type;
}

geometry_msgs::Pose2D Machine::getCenterMachine()
{
	return m_centerMachine;
}

geometry_msgs::Pose2D Machine::getEntryMachine()
{
	return m_entryMachine;
}

geometry_msgs::Pose2D Machine::getExitMachine()
{
	return m_exitMachine;
}

void Machine::majCenter(geometry_msgs::Pose2D point)
{
	m_centerMachine = point;
}

void Machine::majEntry(geometry_msgs::Pose2D point)
{
	m_entryMachine = point;
}

void Machine::majExit(geometry_msgs::Pose2D point)
{
	m_exitMachine = point;
}

manager_msg::activity Machine::msgToGT(int n_robot, int stateOfOrder, int machine, int n_order) // A Verifier
{
	manager_msg::activity msg;
	msg.nb_robot = n_robot;
	msg.state = stateOfOrder;
	msg.machine_used = machine;
	msg.nb_order = n_order;
	return msg;
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
void Machine::take( )
{
	GripperClientSrv gsrv;
	gsrv.gripper_uppdate(true);
}

/* Fonction qui permet de deposer un produit */
void Machine::let( )
{
	GripperClientSrv gsrv;
	gsrv.gripper_uppdate(false);
}

void Machine::startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	DockingClient fa_c;
	fa_c.starting(machineType,machineSide,machineParameter);
}
