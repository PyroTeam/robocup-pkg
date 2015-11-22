#include <feu_tricolore/SimLightDetection.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_light_detection");

    SimLightDetection lightDetection;

    ros::Rate loop_rate(50);
    while (lightDetection.ok()) 
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}