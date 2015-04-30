#ifndef FINALAPPROACHINGACTION_H
#define FINALAPPROACHINGACTION_H

#include <manager_msg/finalApproachingAction.h>

class finalApproachingAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<manager_msg::finalApproachingAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  manager_msg::finalApproachingFeedback feedback_;
  manager_msg::finalApproachingResult result_;
  ros::Publisher m_pub_mvt = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  int m_type;
  int m_side;
  int m_parameter;

public:

  finalApproachingAction(std::string name) :
    as_(nh_, name, boost::bind(&finalApproachingAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~finalApproachingAction(void);

 void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
 int avancement(int a, int b, int c);
 float objectif_x();
 float objectif_y();

};


#endif
