#include "Point.h"
#include "Droite.h"
#include <cmath>
#include <vector>

Droite::Droite(){
	setAngle(0.0);
}

Droite::~Droite(){

}

void Droite::build(Point a, Point b){
  set(a, atan2((b.y - a.y), b.x - a.x));
}