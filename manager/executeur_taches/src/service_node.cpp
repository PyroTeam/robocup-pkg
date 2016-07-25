#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include "GtServerSrv.h"


using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"responding_Generateur_Taches");
    ros::NodeHandle n;

    GtServerSrv gtsrv;
    gtsrv.setId(1);

    ros::ServiceServer service = n.advertiseService("manager/order", &GtServerSrv::responseToGT, &gtsrv);

    ROS_INFO("I'm READY ! ");

    ros::spin();
    return 0;
}
