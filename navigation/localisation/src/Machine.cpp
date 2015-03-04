#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "Machine.h"
#include "line_detection_utils.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Machine::Machine(){
	Point p;
	m_centre = p;
}

Machine::~Machine(){

}

void Machine::build(Modele m){
	Segment s    = m.getSegment();
	Point min    = s.getMin();
	Point max    = s.getMax();
	double angle = s.getAngle();
	double size  = s.getSize();

	double 	gM = 0.70 + 0.02,
			gm = 0.70 - 0.02,
			pM = 0.35 + 0.02,
			pm = 0.35 - 0.02;

	double abscisse, ordonnee;

	//si c'est un grand côté
	if ((size > gm) && (size < gM)){
		abscisse = max.getX()-min.getX()+0.35*sin(M_PI/2-angle);
		ordonnee = max.getY()-min.getY()-0.35*cos(M_PI/2-angle);
		m_orientation = angle;
	}
	//si c'est un petit côté
	if ((size > pm) && (size < pM)){
		abscisse = max.getX()-min.getX()+0.175*sin(M_PI/2-angle);
		ordonnee = max.getY()-min.getY()-0.175*cos(M_PI/2-angle);
		m_orientation = M_PI/2 + angle;
	}

	Point c(abscisse,ordonnee);
	m_centre = c;
}