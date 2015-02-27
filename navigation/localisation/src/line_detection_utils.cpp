

double dist(Point p, Droite d){
	double m, p;
	d.getParam(&m, &p);

	return abs(m*p.x - p.y + p)/sqrt(1+m*m);
}