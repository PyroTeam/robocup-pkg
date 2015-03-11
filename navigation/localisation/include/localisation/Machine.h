#ifndef MACHINE_H
#define MACHINE_H

class Machine{
public:
	Machine(double x = 0.0, double y = 0.0, double theta = 0.0);
	~Machine();

	double getX(){
		return m_x;
	}
	double getY(){
		return m_y;
	}
	double getOrientation(){
		return m_orientation;
	}

	void clear();

private:
	double  m_x;
	double  m_y;
	double  m_orientation;
};

#endif