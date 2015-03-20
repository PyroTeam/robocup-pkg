#include "action.h"
#include <ros/ros.h>
#include <iostream>
#include "robot.h"

using namespace std;

void info_robot(Robot robot){
  cout << "\toccupe: "<<(int)robot.get_occupe()
       << " machine: "<<robot.get_machine()
       << " nb_ordre: "<<robot.get_nb_ordre()<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_node");
  Robot robot[3];
  Action action;
  ros::Rate loop_rate(10);
    
  while (ros::ok())
    {
      for(int i=0;i<3;i++){
	cout << "numero_robot: "<<i<<endl;
	action.update_robot(robot);
	info_robot(robot[i]);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
