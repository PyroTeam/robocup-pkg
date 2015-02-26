#ifndef DROITE_H
#define DROITE_H

class Droite{
public:
	Droite();
	~Droite();

	Point getPoint()                   {return m_point;}
	void getParam(double &a, double &b){a = m_a; b = m_b;}
	double getAngle()                  {return m_angle;}
	double getErreur()                 {return m_erreur;}

	void setPoint(Point point)        {m_point=point;}
	void setParam(double a, double b) {m_a = a; m_b = b;}
	void setAngle(double value)       {m_angle=value;}
	void setErreur(double value)      {m_erreur=value;}

	void initDroite();
	void linReg(std::vector<Point> points);
private:
	Point  m_point;
	double m_angle;
	double m_a;
	double m_b;
};

#endif