#ifndef POINT_H
#define POINT_H

class Point{
public:
	Point(double a, double b);
	~Point();

	double getX(){
		return m_x;
	}
	double getY(){
		return m_y;
	}

private:
	double m_x;
	double m_y;
};

#endif