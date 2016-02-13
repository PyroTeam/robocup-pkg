
/* Classe abstraite qui permet d'aller vers une machine */

#include <string>

#include "Machine.h"

/* Constructeur */
Machine::Machine()
{
	m_centerMachine.x = 0.0;
	m_centerMachine.y = 0.0;
	m_centerMachine.theta = 0.0;
	m_entryMachine.x = 0.0;
	m_entryMachine.y = 0.0;
	m_entryMachine.theta = 0.0;
	m_exitMachine.x = 0.0;
	m_exitMachine.y = 0.0;
	m_exitMachine.theta = 0.0;
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

void getNavigationData(&bool success, &int16_t error, &int16_t status)
{
	success = navigation_success;
	error = navigation_error;
	status = navigation_status;
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

geometry_msgs::Pose2D Machine::zeroPose2D() 
{
	geometry_msgs::Pose2D point
	point.x = 0;
	point.y = 0;
	point.theta = 0;
	return point;
}

/* Fonction abstraite qui permet d'aller vers une machine (Point centre/entree/sortie d'une machine) */

void Machine::goTo(geometry_msgs::Pose2D pt_dest, bool fast, bool puckInGripper, bool goAway)
{
	ROS_INFO("going to the point : x %f - y %f - theta %f",pt_dest.x,pt_dest.y,pt_dest.theta);
	NavigationClientAction n_c;
	int stateOfNavigation = n_c.navigate(pt_dest,false,fast,puckInGripper,goAway,0,0,0);
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

void Machine::readlights(std::vector<comm_msg::LightSpec> lSpec)
{
	ROS_INFO(" Starting exploring the lights ");
	FeuClientAction f_c;
	f_c.lightsStates(lSpec);
	ROS_INFO("end of exploring the lights");
}

void Machine::startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter)
{
	geometry_msgs::Pose2D point = zeroPose2D();
	NavigationClientAction n_c;
	n_c.navigate(point,true,false,false,false,machineType,machineSide,machineParameter);
}
