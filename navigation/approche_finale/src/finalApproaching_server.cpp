#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
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

public:

  finalApproachingAction(std::string name) :
    as_(nh_, name, boost::bind(&finalApproachingAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~finalApproachingAction(void)
  {
  }

  void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    feedback_.percent_complete = 0;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating finalApproaching sequence of type %i with side %i and parameter %i", action_name_.c_str(), goal->type, goal->side,goal->parameter);

    // start executing the action
    for(int i=1; i<=100; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.percent_complete++;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "finalApproaching");

  finalApproachingAction finalApproaching(ros::this_node::getName());
  ros::spin();

  return 0;
}

