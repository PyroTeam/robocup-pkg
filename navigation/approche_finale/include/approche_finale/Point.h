// d apres le code de Thomas Danel

#ifndef POINT_H
#define POINT_H

class Point{
public:

Point(float r = 0.0, float phi = 0.0);
~Point();

void setr(float val){m_r = val;}
void setphi(float val){m_phi = val;}
float getr() const{return m_r;}
float getphi() const{return m_phi;}

private:

float m_r;
float m_phi;

};

#endif
