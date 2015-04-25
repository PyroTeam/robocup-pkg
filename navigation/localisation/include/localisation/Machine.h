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
	void setType(int val){
		m_type = val;
	}
	void resetType(){
		m_type = 0;
	}

private:
	geometry_msgs::Pose2D 	m_centre;
	int						m_type;
};

#endif