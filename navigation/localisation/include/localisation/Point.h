#ifndef POINT_H
#define POINT_H

class Point{
public:
	Point(double a = 0.0, double b = 0.0);
	~Point();

	void setX(double val){
		m_x = val;
	}
	void setY(double val){
		m_y = val;
	}
	double getX() const{
		return m_x;
	}
	double getY() const{
		return m_y;
	}

private:
	double m_x;
	double m_y;
};

#endif