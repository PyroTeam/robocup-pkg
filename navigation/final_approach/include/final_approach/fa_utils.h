/**
 * \file         fa_utils.h
 * \brie         ensemble de fonctions nécessaires pour l'approche finale
 * \author       Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date         2015-04-20
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _FINAL_APPROACH__FA_UTILS__H_
#define _FINAL_APPROACH__FA_UTILS__H_

#include "final_approach/Point.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <final_approach_msg/plotDataFA.h>

#include <vector>


geometry_msgs::Point orthoProjection(Point p, geometry_msgs::Pose2D p2d);
/**
 *  \brief		Calcule la distance en m entre deux points
 *  \return		distance en m entre deux points
 */
float distance2points(Point a, Point b);

/**
 *  \brief		Fait une moyenne
 *  \return		la valeur moyenne de l'ensemble des flottants passes en parametres
 */
float moy(std::list<float> positionY);


#endif // _FINAL_APPROACH__FA_UTILS__H_
