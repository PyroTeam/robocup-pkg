#include "Segment.h"
#include "line_detection_utils.h"

void Segment::build(const std::list<Point> &points){
	//on calcule les coordonnées des projetés orthogonaux des deux points extrêmes
	Point a2 = ortho(*(points.begin()));
	Point b2 = ortho(*(points.end()));
	//...puis la taille du segment en mètre
	double size = sqrt( (a2.getX()-b2.getX()) * (a2.getX()-b2.getX()) +
						(a2.getY()-b2.getY()) * (a2.getY()-b2.getY()));

	setPoints(a2,b2);
	setSize(size);
}

double Segment::dist(Point a){
  Point b = getMin();
  Point u(cos(getAngle()), sin(getAngle()));
  Point ba(a.getX()-b.getX(), a.getY()-b.getY());

  return std::abs(u.getX()*ba.getY() - ba.getX()*u.getY()) / sqrt(u.getX()*u.getX() + u.getY()*u.getY());
}
Point Segment::ortho(Point a){
  double distance = dist(a);
  double dx = distance*cos(getAngle());
  double dy = distance*sin(getAngle());
  Point p(a.getX() + dx, a.getY() + dy);

  return p;
}