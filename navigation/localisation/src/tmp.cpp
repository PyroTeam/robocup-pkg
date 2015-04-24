int machineToArea(geometry_msgs::Pose2D m){
  int zone = 0, x = 0, y = 0;

  //getZone(m)
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

  if (zone == 0){
    return 0;
  }

  //getCenter(zone)
  if(zone > 0){
    // Right side
    if(zone<=12) {
      x = ((zone-1)/4)*2 + 1;
      y = ((zone-1)%4)*1.5 + 0.75;
    }
    // Left side
    else if (zone<=24) {
      zone -=12;
      x = -((zone-1)/4)*2 - 1;
      y = ((zone-1)%4)*1.5 + 0.75;
    }
  }

  //dist(m,p)
  if (zone != 0){
    double d = (m.x - x)*(m.x - x) + (m.y - y)*(m.y - y);
  }
  //si on est dans le cercle de centre le centre de zone et de rayon 0.6 m
  //pour éviter un sqrt() on met le seuil au carré
  if (d <= 0.36){
    return zone;
  }
  else {
    return 0;
  }
}