#include <feu_tricolore/cv_algo.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lecture_feu");
    LectureFeu lf;
    ros::Rate loop_rate(4);
    while (lf.ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}