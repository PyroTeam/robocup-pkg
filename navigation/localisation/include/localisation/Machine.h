#ifndef MACHINE_H
#define MACHINE_H

class Machine{
public:
	Machine();
	~Machine();

	Point getCentre(){
		return m_centre;
	}
	double getOrientation(){
		return m_orientation;
	}

	void set(Point p, double theta){
		m_point       = p;
		m_orientation = theta;
	}

	void build(Modele m);

private:
	Point   m_centre;
	double  m_orientation;
};

#endif