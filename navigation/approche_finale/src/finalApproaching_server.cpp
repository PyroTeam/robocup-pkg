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
    ros::Rate loop_rate(1);
    bool success = true;

    feedback_.percent_complete = 0;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating finalApproaching sequence of type %i with side %i and parameter %i", action_name_.c_str(), goal->type, goal->side,goal->parameter);
    int type, side, parameter; 
    nh_.param<int>("fa_type",type,goal->type);
    nh_.param<int>("fa_side",side,goal->side);
    nh_.param<int>("fa_parameter",parameter,goal->parameter);
    
    /*
    bool objectif=false
    while(objectif==false){
        while(deplacement_msg::landmarks[2] > 1.60 || geometry_msgs/Pose2D[2] < 1.54){
           if(deplacement_msg::landmarks[2] > 1.60){
              //tourner dans le sens anti trigo   
           }
           if(deplacement_msg::landmarks[2] < 1.54){
              //tourner dans le sens trigo  
           }
        }
        percent_complete = 20;
        
        if(ARTAG.cote != side){
           success = false;
           break;
        }
        
        percent_complete = 40;
        
        while(deplacement_msg::landmarks[1] > 0.20){ //distance en metre
             //avancer vers machine seulement suivant l'axe y   
        }
        percent_complete = 60;
        
        while(deplacement_msg::landmarks[0]){
                if(position(message scan,position robot)==1)
                        aller a droite
                if(position(message scan,position robot)==2)
                        aller a gauche
        }
        percent_complete = 80;
        
        while(distance(laser,machine) > 0.05){
             avancer vers machine   
        }
        percent_complete = 100;
        objectif=true;   
    }
    */
   
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
      //tasks to do 
      // publish the feedback
      as_.publishFeedback(feedback_);
      
      
      loop_rate.sleep();
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
  ROS_INFO("Initialisation du serveur");

  finalApproachingAction finalApproaching(ros::this_node::getName());
  ROS_INFO("test");
  ros::spin();

  return 0;
}

