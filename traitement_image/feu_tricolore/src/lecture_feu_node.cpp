#include <feu_tricolore/cv_algo.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lecture_feu");
    LectureFeu lf;
    ros::Rate loop_rate(50);
    while (lf.ok()) 
    {   
        // Mise a jour des topics et Cie
        ros::spinOnce();

        // TODO - Traitement

        // Pause
        loop_rate.sleep();
    }
    return 0;
}