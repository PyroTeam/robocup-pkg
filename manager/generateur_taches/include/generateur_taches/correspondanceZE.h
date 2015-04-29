#ifndef CORRESPONDANCEZE_H
#define CORRESPONDANCEZE_H

#include "comm_msg/ExplorationInfo.h"
#include <ros/ros.h>
#include <vector>

class CorrespondanceZE {

	public:

	CorrespondanceZE();
	~CorrespondanceZE();
	void cZECallback(const comm_msg::ExplorationInfo &msg);
	std::vector<int> getUsefulZone();

	private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_correspondanceZESub;
	std::vector<int> m_usefulZone;
};

#endif
