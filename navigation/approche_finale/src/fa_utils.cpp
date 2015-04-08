#include <cmath>

#include "fa_utils.h"
#include "Point.h"

float distance2points(Point a, Point b){
float xa = a.getr()*cos(a.getphi());
float xb = b.getr()*cos(b.getphi());
float ya = a.getr()*sin(a.getphi());
float yb = b.getr()*sin(b.getphi());
return sqrt( (xb-xa)*(xb-xa) + (yb-ya)*(yb-ya) );
}
