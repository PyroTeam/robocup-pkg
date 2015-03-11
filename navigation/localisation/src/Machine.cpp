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

Machine::Machine(double a, double b, double c) : m_x(a), m_y(b), m_orientation(c){
	
}

Machine::~Machine(){

}

void Machine::clear(){
	m_x 		  = 0.0;
	m_y 		  = 0.0;
	m_orientation = 0.0;
}