#include <cmath>

double dist(Point a, Droite d){
	Point b = d.getPoint();

	return abs(a.x*b.y - b.x*a.y) / sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}