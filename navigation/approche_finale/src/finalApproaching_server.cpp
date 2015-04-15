#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <manager_msg/finalApproachingAction.h>
#include <cmath>
#include <vector>
#include <list>

#include "fa_utils.h"
#include "laserScan.h"
#include "Point.h"

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
    ros::Rate loop_rate(5);
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
	ros::NodeHandle n;
	ros::Time::init();
	ros::Rate loop_rate(1000);
	laserScan ls;
	loop_rate.sleep();
	std::vector<float> position_y(100);
	std::vector<float> pente(100);
	int max_points=0;
	int nearby=0;
	float d=0;
	int j=0;
	float pos=0;
	std::cout << "En attente d'un scan laser complet" << std::endl;
	while(ros::ok()){
		
    if(ls.getRanges().size() == 513){
			max_points = ls.max_number_points();
			nearby = ls.nearest_object();
			d = ls.distance_objet(max_points);
			pos = ls.position_y(nearby,d);
			Segment s = ls.build_segment(nearby);
			if(j < position_y.size()){
				if(pos > 0){
					position_y[j] = ls.position_y(nearby,d);
					j++;
					}
			}
			else{				
				if(pos>0){
					s = ls.build_segment(max_points);
					for(int i=0;i<position_y.size()-1;i++){
						position_y[i] = position_y[i+1];
						pente[i] = pente[i+1];
					}
					position_y[position_y.size()-1] = pos;
					pente[pente.size()-1] = s.get_pente();
					
				}
			}
			std::cout << "le laser se situe a " << moy(position_y) <<" m du bord gauche de l'objet le plus proche" << std::endl;
			std::cout << "pente = " << moy(pente) << std::endl;
			asservissement_angle(n,moy(pente));
			
			}
		}
		
		ros::spinOnce();
    loop_rate.sleep();
	
		/*
		//pour l instant 15 cm rentré en dur
		float distance_x = s.get_min().getr()*cos(s.get_min().getphi());
		while((distance_x < 0.1505) && (distance_x > 0.1495)){ 
			if(distance_x < 0.1505){
				ROS_INFO("on avance vers la gauche\n");
				//ordonner au moteur d'aller a gauche
			}
			if(distance_x > 0.1495){
				ROS_INFO("on avance vers la droite\n");
				//ordonner au moteur d'aller a droite
			}
			max_points=ls.max_number_points();
			if(ls.around_70cm(max_points) && ls.nearby_object(max_points))
				ROS_INFO("conditions reunies pour creer nouveau segment\n");
			else
				ROS_INFO("conditions non reunies pour creer nouveau segment\n");
    	}
	
	*/
	
	/*
  finalApproachingAction finalApproaching(ros::this_node::getName());
  ROS_INFO("test");
  ros::spin();
	*/
}
