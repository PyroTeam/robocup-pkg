#include <feu_tricolore/cv_algo.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "light_detection");
    LectureFeu lf;
    ros::Rate loop_rate(50);

    while (lf.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}