/**
 * \file        BasicGradientModifier.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-28
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */
#ifndef OCCUPANCY_GRID_UTILS_BASICGRADIENTMODIFIER_H_
#define OCCUPANCY_GRID_UTILS_BASICGRADIENTMODIFIER_H_

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include <nav_msgs/OccupancyGrid.h>
#include "GridModifier.h"

namespace occupancy_grid_utils {


/**
 * \class BasicGradientModifier
 * \brief Classe permettant d'ajouter un dégradé autour des objets de la grid map
 *
 * Cette classe détermine des poids dégradé autour des objets d'une OccupancyGrid,
 * en ce basant sur le principe de transformation de distance en traitement d'image.
 * Cette classe fournit à la fois une interface pour ces algorithmes, ainsi qu'une
 * implémentation basique.
 * Cette implémentation donne des résultats exacts mais n'a pas pour but d'être
 * utilisée en match parce qu'elle est très gourmande en temps de calcul,
 * en revanche elle servira de référence s'il est nécessaire de comparer
 * les résultats d'un algorithme plus rapide mais basé sur une approximation.
 *
 */
class BasicGradientModifier : public GridModifier
{
public:
    BasicGradientModifier(double distance = 0.05, unsigned int minValue = 0):GridModifier()
    {
        setParameters(distance, minValue);
    }
    virtual ~BasicGradientModifier()
    {

    }

    void setParameters(double distance, unsigned int minValue);
protected:
    double m_distance;
    unsigned int m_minValue;
    double m_a;
    
    virtual void executeImpl(nav_msgs::OccupancyGrid &grid) override;
};



} // namespace occupancy_grid_utils

#endif /* OCCUPANCY_GRID_UTILS_FASTGRADIENTMODIFIER_H_ */
