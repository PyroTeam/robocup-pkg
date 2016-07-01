#ifndef _COMMON_UTILS__ZONE__H_
#define _COMMON_UTILS__ZONE__H_


#include "geometry_msgs/Pose2D.h"

namespace common_utils {

/**
 * \brief      Donne les coordonées (x, y) du centre d'une zone
 *
 * \details    Calcul les coordonées du centre d'une zone pour une map de 24 zones. Cette map
 *             ayant une configuration simulaire à la Robocup Logistic League 2016. La largeur et la
 *             hauteur d'une zone sont paramétrables.
 *
 * \param[in]  zone         The zone (must be 1 to 24)
 * \param[out] x            x coordinate
 * \param[out] y            y coordinate
 * \param[in]  zone_width   The zone width
 * \param[in]  zone_height  The zone height
 *
 * \return     true if success, false otherwise.
 */
inline bool getZoneCenter(int zone, double &x, double &y, float zone_width = 1.96, float zone_height = 1.5)
{
	bool leftSide = zone > 12;

	if (zone < 1 || zone > 24)
	{
		ROS_ERROR("Wrong zone [%d], must be 1 to 24", zone);
		x = 0;
		y = 0;
		return false;
	}

	// Get zone center
	zone = (leftSide)? zone-12 : zone;
	x = ((zone-1)/4)*zone_width + zone_width/2;
	y = ((zone-1)%4)*zone_height + zone_height/2;
	if (leftSide)
	{
		x *= -1;
	}

	return true;
}

inline int getArea(const geometry_msgs::Pose2D &m, float zone_width = 1.96, float zone_height = 1.5)
{
  int area = 0;

  // Right side
  if(m.x >= 0 && m.x <= 6 && m.y >= 0 && m.y < 6)
  {
    int w = int(m.x/zone_width);
    int h = int(m.y/zone_height)+1;

    area =  w*4 + h;
  }
  // Left Side
  else if (m.x >= -6 && m.x < 0 && m.y >= 0 && m.y < 6)
  {
    int w = int(-m.x/zone_width);
    int h = int(m.y/zone_height)+1;

    area =  w*4 + h + 12;
  }

  return area;
}

} // namespace common_utils

#endif // _COMMON_UTILS__ZONE__H_
