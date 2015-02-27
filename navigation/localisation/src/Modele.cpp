#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include <time.h>
#include <cmath>
#include <limits>

Modele::Modele(){
	double inf = std::numeric_limits<double>::max();
	setErreur(inf);
}

Modele::~Modele(){

}

void Modele::linReg(){
	m_erreur = m_droite.linReg(m_points);
}

void Modele::build(Point a, Point b){
  //on fabrique une droite...
  Droite d;
  d.set(a, atan2((b.y - a.y), b.x - a.x));
  
  //...pour fabriquer un modele possible
  setDroite(d);
  addPoint(a);
  addPoint(b);
}