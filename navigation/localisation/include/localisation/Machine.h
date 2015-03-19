#ifndef MACHINE_H
#define MACHINE_H

#include "geometry_msgs/Pose2D.h"

class Machine{
public:
	Machine();
	~Machine();

	geometry_msgs::Pose2D getCentre(){
		return m_centre;
	}
	bool getType(){
		return m_type;
	}

	void setCentre(geometry_msgs::Pose2D c){
		m_centre = c;
	}
	void setOrientation(double theta);
	void setType(){
		m_type = true;
	}
	void resetType(){
		m_type = false;
	}

private:
	geometry_msgs::Pose2D 	m_centre;
	bool					m_type;
};

#endif