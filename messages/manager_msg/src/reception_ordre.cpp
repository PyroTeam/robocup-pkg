#include "ros/ros.h"
#include "manager_msg/ordre.h"

bool add(manager_msg::ordre::Request  &req,
         manager_msg::ordre::Response &res)
{
 res.id = req.id;
 res.ordre = req.ordre;
 ROS_INFO("request: Numero Robot=%d, Ordre=%d, Parametre=%d, Id=%d",(int)req.numero_robot,(int)req.ordre,(int)req.parametre,(int)req.id);
 ROS_INFO("etat:1, Ordre=%d Id=%d",/*(int)res.accepte,*/(int)res.ordre,(int)res.id);
 return true;
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "reception_ordre");
 ros::NodeHandle n;

 ros::ServiceServer service = n.advertiseService("ordre", add);
 ROS_INFO("Ready to send order.");
 ros::spin();

 return 0;
}
