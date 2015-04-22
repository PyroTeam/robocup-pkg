/** 
* \file         fakeRobotinoNode.cpp
*
* \brief        noeud de test simulant certaines fonctions, 
*               non disponible actuellement sur le robotino
*
* \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
* \date         2015-03-17
* \copyright    PyroTeam, Polytech-Lille
* \license
* \version
*/


#include "fakeRobotino.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_robotino_node");
    
    FakeRobotino fakeRobotino;
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        fakeRobotino.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
