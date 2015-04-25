// d apres le code de Thomas Danel

#ifndef POINT_H
#define POINT_H

#include <cmath>

class Point{
public:

Point(float r = 0.0, double phi = 0.0);
~Point();

void setr(float val){m_r = val;}
void setphi(double val){m_phi = val;}
float getr() const{return m_r;}
double getphi() const{return m_phi;}
float getx() const{return m_r*cos(-M_PI_2 + m_phi);}
float gety() const{return m_r*sin(-M_PI_2 + m_phi);}
private:

float m_r;
double m_phi;

};

#endif
