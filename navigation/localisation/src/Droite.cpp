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
	double pente = (b.getY() - a.getY())/(b.getX() - a.getX());
	double ordOrigin = (b.getY() - a.getY()) - pente * (b.getX() - a.getX());

  	set(a,
  		atan2(pente,1),
  		pente,
  		ordOrigin);
}