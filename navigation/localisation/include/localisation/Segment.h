#ifndef SEGMENT_H
#define SEGMENT_H

#include "Point.h"

#include <vector>
#include <list>
#include <cmath>

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
		m_angle = atan2(tan(theta), 1);
	}
	void setSize(double size){
		m_size = size;
	}
	void setPoints(Point a, Point b){
		m_min = a;
		m_max = b;
	}
	void update(Segment s){
		if (s.getMin().getX() < m_min.getX()){
			m_min = s.getMin();
		}
		if (s.getMax().getX() < m_max.getX()){
			m_max = s.getMax();
		}

		m_size = sqrt((m_max.getX()-m_min.getX()) * (m_max.getX()-m_min.getX()) +
                      (m_max.getY()-m_min.getY()) * (m_max.getY()-m_min.getY()));

		m_angle = (s.getAngle() + m_angle) / 2;
	}
	
protected:
	double			 m_angle;
	double 			 m_size;
	Point   		 m_min;
	Point   		 m_max;
};

#endif