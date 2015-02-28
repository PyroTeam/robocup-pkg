#include "Point.h"
#include "Droite.h"
#include <cmath>

Droite::Droite(){
	setAngle(0.0);
}

Droite::~Droite(){

}

Droite::build(Point a, Point b){
  set(a, atan2((b.y - a.y), b.x - a.x));
}