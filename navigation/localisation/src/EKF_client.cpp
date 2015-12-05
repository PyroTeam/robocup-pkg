#include "ros/ros.h"
#include "deplacement_msg/Alarm.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EKF_client");
  if (argc != 1)
  {
    ROS_INFO("usage: 1 to begin EKF, 0 to end");
    //return 1;
  }

  std::cout << "argument = " << argv[1] << std::endl;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<deplacement_msg::Alarm>("objectDetection/wake_up");
  deplacement_msg::Alarm srv;

  srv.request.wake_up = atoi(argv[1]);

  if (client.call(srv))
  {
    if(atoi(argv[1]) == 1)
    {
      std::cout << "Je vais me réveiller" << std::endl;
    }
    else if (atoi(argv[1]) == 2){
      std::cout << "Je vais me coucher" << std::endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}