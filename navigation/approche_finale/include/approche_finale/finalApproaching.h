#ifndef FINALAPPROACHING_H
#define FINALAPPROACHING_H

#include <manager_msg/finalApproachingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

class finalApproaching
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<manager_msg::finalApproachingAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  manager_msg::finalApproachingFeedback feedback_;
  manager_msg::finalApproachingResult result_;
  ros::Publisher m_pub_mvt;
  int m_type;
  int m_side;
  int m_parameter;

public:

  finalApproaching(std::string name) :
    as_(nh_, name, boost::bind(&finalApproaching::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~finalApproaching(void);

 void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
 int avancement(int a, int b, int c);
 float objectif_x();
 float objectif_y();

};


#endif
