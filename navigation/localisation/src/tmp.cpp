int getZone(geometry_msgs::Pose2D m){
  //côté droit
  if(m.x >= 0 && m.y >= 0 && m.x <= 6 && m.y < 6) {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    zone = w*4 + h;
  }
  //côté gauche
  else if (m.x < 0 && m.y >= 0 && m.x <= -6 && m.y < 6) {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    zone = w*4 + h + 12;
  }
  else {
    zone = 0;
  }

  return zone;
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
  //si on est dans le cercle de centre le centre de zone et de rayon 0.6 m
  //pour éviter un sqrt() on met le seuil au carré
  if (dist(m,getCenter(getZone(m))) <= 0.36){
    return zone;
  }
  else {
    return 0;
  }
}