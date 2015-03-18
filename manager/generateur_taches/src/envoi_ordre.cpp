#include "ros/ros.h"
#include "manager_msg/ordre.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "envoi_ordre");
  if (argc != 6)
    {
      ROS_INFO("usage: envoi_ordre numero_robot numero_ordre type parametre id");
      return 1;
    }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manager_msg::ordre>("ordre");
  manager_msg::ordre srv;
  
  srv.request.numero_robot = atoll(argv[1]);
  srv.request.numero_ordre = atoll(argv[2]);
  srv.request.type = atoll(argv[3]);
  srv.request.parametre = atoll(argv[4]);
  srv.request.id = atoll(argv[5]);
  
  srv.response.id = atoll(argv[5]);
  srv.response.id = 1;
  if (client.call(srv))
    {
      ROS_INFO("Etat: %d, Id: %d", (int)srv.response.accepte, (int)srv.response.id);
    }
  else
    {
      ROS_ERROR("Failed to call service ordre");
      return 1;
    }

  return 0;
}
