#ifndef DROITE_H
#define DROITE_H

class Droite{
public:
	Droite();
	~Droite();

	Point getPoint(){
		return m_point;
	}
	double getAngle(){
		return m_angle;
	}

	void set(Point p, double theta){
		m_point = p;
		m_angle = theta;
	}

	void build(Point a, Point b);

private:
	Point  m_point;
	double m_angle;
};

#endif