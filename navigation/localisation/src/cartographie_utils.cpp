#include "cartographie_utils.h"

// Field = 6 m x 12 m
int getArea(geometry_msgs::Pose2D m)
{
  // Right side
  if(m.x >= 0 && m.x <= 6 && m.y >= 0 && m.y < 6)
  {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h;
  }
  // Left Side
  else if (m.x >= -6 && m.x < 0 && m.y >= 0 && m.y < 6)
  {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h + 12;
  }
  else
  {
    return 0;
  }
}

// Area = 2 m x 1.5 m
geometry_msgs::Pose2D getCenter(int area)
{
  geometry_msgs::Pose2D c;

  // Right side
  if(area<=12)
  {
    c.x = ((area-1)/4)*2 + 1;
    c.y = ((area-1)%4)*1.5 + 0.75;
  }
  // Left side
  else if (area<=24)
  {
    area -=12;
    c.x = -((area-1)/4)*2 - 1;
    c.y = ((area-1)%4)*1.5 + 0.75;
  }

  return c;
}
