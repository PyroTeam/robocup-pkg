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

Machine::Machine() : m_xSum(0.0),m_ySum(0.0),m_thetaSum(0.0),m_nbActu(0.0),m_state(false){

};

Machine::~Machine(){

}

geometry_msgs::Pose2D Machine::getCentre(){
	return m_centre;
}
bool Machine::getType(){
	return m_type;
}

void Machine::setCentre(geometry_msgs::Pose2D c){
	m_centre = c;
}
void Machine::setType(int val){
	m_type = val;
}
void Machine::resetType(){
	m_type = 0;
}
void Machine::addX(double x){
	m_xSum = x;
	// m_xSum += x;
}
void Machine::addY(double y){
	m_ySum = y;
	// m_ySum += y;
}
void Machine::addTheta(double theta){
	m_thetaSum = theta;
	// m_thetaSum += theta;
}
void Machine::incNbActu(){
	m_nbActu = 1;
	// m_nbActu ++;
}

void Machine::maj(){
	m_centre.x = m_xSum/m_nbActu;
	m_centre.y = m_ySum/m_nbActu;
	m_centre.theta = m_thetaSum/m_nbActu;

	m_state = true;
}
bool Machine::exist(){
	return m_state;
}