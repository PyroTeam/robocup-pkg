#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"


class Machine{
  protected:
 
    /* Variables d'instance */
    std::string m_type;
    geometry_msgs::Pose2D m_centreMachine;
    geometry_msgs::Pose2D m_entreeMachine;
    geometry_msgs::Pose2D m_sortieMachine;
    
  public:
    Machine();
    virtual ~Machine();
    
    virtual void FonctionVirtuelle() = 0;
    
    /* méthodes */
    std::string getType();
    geometry_msgs::Pose2D getCentreMachine();
    geometry_msgs::Pose2D getEntreeMachine();
    geometry_msgs::Pose2D getSortieMachine();
    void majEntree(geometry_msgs::Pose2D point);
    void majSortie(geometry_msgs::Pose2D point);
    
    
};

#endif