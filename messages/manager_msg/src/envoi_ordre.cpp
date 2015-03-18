#include "ros/ros.h"
#include "manager_msg/ordre.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "envoi_ordre");
  if (argc != 5)
    {
      ROS_INFO("usage: envoi_ordre numero_robot ordre parametre id");
      return 1;
    }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manager_msg::ordre>("ordre");
  manager_msg::ordre srv;
  srv.request.numero_robot = atoll(argv[1]);
  srv.request.ordre = atoll(argv[2]);
  srv.request.parametre = atoll(argv[3]);
  srv.request.id = atoll(argv[4]);
  srv.response.id = atoll(argv[4]);
  if (client.call(srv))
    {
      ROS_INFO("Etat: 1, Id: %d", /*(int)srv.response.accepte,*/ (int)srv.response.id);
    }
  else
    {
      ROS_ERROR("Failed to call service ordre");
      return 1;
    }

  return 0;
}
