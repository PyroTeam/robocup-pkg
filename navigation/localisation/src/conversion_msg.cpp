#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <list>
#include <algorithm>

#include "line_detection_utils.h"

geometry_msgs::Pose2D convertMachineToPose2D(Machine m){
  geometry_msgs::Pose2D pose2d;

  pose2d.x     = m.getX();
  pose2d.y     = m.getY();
  pose2d.theta = m.getOrientation();

  return pose2d;
}