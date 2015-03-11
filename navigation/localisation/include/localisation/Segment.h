#ifndef SEGMENT_H
#define SEGMENT_H

#include "Point.h"

#include <vector>
#include <list>

class Segment{
public:
	Segment() : m_angle(0.0),m_size(0.0){

	}
	~Segment(){

	}

	double getAngle(){
		return m_angle;
	}
	double getSize(){
		return m_size;
	}
	Point getMin(){
		return m_min;
	}
	Point getMax(){
		return m_max;
	}

	void setAngle(double theta){
		m_angle = theta;
	}
	void setSize(double size){
		m_size = size;
	}
	void setPoints(Point a, Point b){
		m_min = a;
		m_max = b;
	}
	
private:
	double			 m_angle;
	double 			 m_size;
	Point   		 m_min;
	Point   		 m_max;
};

#endif