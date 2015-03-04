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
	double getPente(){
		return m_pente;
	}
	double getOrdOrigin(){
		return m_ordOrigin;
	}

	void set(Point p, double theta, double pente, double ord){
		m_point = p;
		m_angle = theta;
		m_pente = pente;
		m_ordOrigin = ord;
	}
	void setAngle(double theta){
		m_angle = theta;
	}

	void build(Point a, Point b);

private:
	Point  m_point;
	double m_pente;
	double m_ordOrigin;
	double m_angle;
};

#endif