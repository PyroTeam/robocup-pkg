#include "ros/ros.h"
 #include "std_msgs/String.h"
 #include "manager_msg/activity.h" 

 #include <sstream>
 using namespace manager_msg;
 int main(int argc, char **argv)
 {
   ROS_INFO("Starting node GT Publisher");
   ros::init(argc, argv, "gtPublisher");
   ros::NodeHandle n;
   ros::Publisher activite = n.advertise<activity>("/task_exec_state", 1000);
   ros::Rate loop_rate(50);
   int count = 0;
   while (ros::ok())
     {
       activity msg;
       msg.nb_robot = 1;
       msg.state = activity::IN_PROGRESS; /* Par exemple */
       if(count>20)
         msg.nb_robot = 2;
       activite.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
       ++count;
     }
   return 0;
 }