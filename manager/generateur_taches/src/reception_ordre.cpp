#include "ros/ros.h"
#include "manager_msg/ordre.h"

bool add(manager_msg::ordre::Request  &req,
         manager_msg::ordre::Response &res)
{
 res.id = req.id;
 res.numero_ordre = req.numero_ordre;
 res.numero_robot = req.numero_robot;
 res.accepte = 1;
 ROS_INFO("request: Numero Robot=%d, Numero_ordre=%d, Type=%d, Parametre=%d, Id=%d",(int)req.numero_robot,(int)req.numero_ordre,(int)req.type,(int)req.parametre,(int)req.id);
 ROS_INFO("Etat:%d, Numero_robot=%d, Numero_ordre=%d Id=%d",(int)res.accepte,(int)res.numero_robot,(int)res.numero_ordre,(int)res.id);
 return true;
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "reception_ordre");
 ros::NodeHandle n;

 ros::ServiceServer service = n.advertiseService("ordre", add);
 ROS_INFO("Pret a recevoir des ordres.");
 ros::spin();

 return 0;
}
