#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Landmarks.h"
#include "Machine.h"
#include <cmath>

int getZone(geometry_msgs::Pose2D m){
  //côté droit
  if(m.x >= 0 && m.y >= 0 && m.x <= 6 && m.y < 6) {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h;
  }
  //côté gauche
  else if (m.x < 0 && m.y >= 0 && m.x <= -6 && m.y < 6) {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h + 12;
  }
  else {
    return 0;
  }
}

geometry_msgs::Pose2D getCenter(int zone){
  geometry_msgs::Pose2D c;

  // Right side
  if(zone<=12) {
    c.x = ((zone-1)/4)*2 + 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }
  // Left side
  else if (zone<=24) {
    zone -=12;
    c.x = -((zone-1)/4)*2 - 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }

  return c;
}

double dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m){
  return (m.x - c.x)*(m.x - c.x) + (m.y - c.y)*(m.y - c.y);
}

int machineToArea(geometry_msgs::Pose2D m){
  int zone = getZone(m);
  std::cout << "machine (" << m.x << "," << m.y << ") en zone " << zone << std::endl;
  if ((zone != 0) && (dist(m,getCenter(zone)) <= 0.36)){
  //si on est dans le cercle de centre le centre de zone et de rayon 0.6 m
  //pour éviter un sqrt() on met le seuil au carré
    return zone;
  }
  else {
    return 0;
  }
}

deplacement_msg::Landmarks convert(std::vector<Machine> mps){
  deplacement_msg::Landmarks tmp;

  for (auto &it : mps){
    if (it.exist()){
      tmp.landmarks.push_back(it.getCentre());
    }
  }

  return tmp;
}