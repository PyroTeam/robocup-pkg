#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "manager_msg/activity.h"
#include "manager_msg/order.h"
#include "manager_msg/MoveToPoseAction.h"
#include "FeuClientAction.h"
#include "GripperClientSrv.h"
#include "FinalApproachingClient.h"
#include "ExploInfoSubscriber.h"
#include "NavigationClientAction.h"

using namespace manager_msg;

class Machine{
  protected:
 
    /* Variables d'instance */
    std::string m_type;
    geometry_msgs::Pose2D m_centerMachine;
    geometry_msgs::Pose2D m_entryMachine;
    geometry_msgs::Pose2D m_exitMachine;
    int zone;
    bool isHere;
    
  public:
    Machine();
    virtual ~Machine();
    
    virtual void FonctionVirtuelle() = 0;
    
    /* méthodes */
    std::string getType();
    geometry_msgs::Pose2D getCenterMachine();
    geometry_msgs::Pose2D getEntryMachine();
    geometry_msgs::Pose2D getExitMachine();
    void majEntry(geometry_msgs::Pose2D point);
    void majExit(geometry_msgs::Pose2D point);
    manager_msg::activity msgToGT(int n_robot, int stateOfOrder, int machine, int n_order);
    void goTo(geometry_msgs::Pose2D pt_dest);
    void take();
    void let();
    void readlights(std::vector<manager_msg::LightSpec> lSpec);
    void startFinalAp(int8_t machineType, int8_t machineSide, int8_t machineParameter);

};

#endif



