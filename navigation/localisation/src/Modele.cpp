#include "Point.h"
#include "Droite.h"
#include "Modele.h"
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