#include "MPS.h"
#include "geometry_msgs/Pose2D.h"

/* Constructeur */
MPS::MPS()
{
	x = 0;
    y = 0;
	theta  = 0;
	zone   = -1;
	isHere = false;
}

/* Destructeur */
MPS::~MPS(){}