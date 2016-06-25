#include "LocaSubscriber.h"

//std::vector<MPS> machine(24);

LocaSubscriber::LocaSubscriber(std::list<int> &lst_unkownZones, std::list<int> &lst_exploredZones, std::list<int> &lst_notExploredZones)
: m_unkownZones(lst_unkownZones)
, m_exploredZones(lst_exploredZones)
, m_notExploredZones(lst_notExploredZones)
{
	ros::NodeHandle n;
	m_sub = n.subscribe("objectDetection/landmarks",1000,&LocaSubscriber::tesCallback, this);
}

LocaSubscriber::~LocaSubscriber(){}

void LocaSubscriber::tesCallback(const deplacement_msg::MachinesConstPtr &msg)
{
	ROS_INFO_ONCE("I heard the localisation publisher ");

	m_machinesPose = msg->landmarks;

	for(auto &m : m_machinesPose)
	{
		  int actualZone = m.zone;
			for(auto &uZ : m_unkownZones)
			{
				if(uZ == actualZone)
				{
					m_unkownZones.remove(uZ);

					bool foundInExp = foundInExplored(uZ);
					bool foundInNotExp = foundInNotExplored(uZ);
					if(!foundInExp && !foundInNotExp)
					{
						m_notExploredZones.push_back(uZ);
					}
				}
			}
			}
	// for (int i=0; i< msg->landmarks.size(); i++)
	// {
	// 	int zone = getZone(msg->landmarks[i].x, msg->landmarks[i].y);
	// 	m_machine[zone-1].x = msg->landmarks[i].x;
	// 	m_machine[zone-1].y = msg->landmarks[i].y;
	// 	m_machine[zone-1].theta = msg->landmarks[i].theta;
	// 	m_machine[zone-1].isHere = true;
	// 	m_machine[zone-1].zone = zone;
	// }
	/*for(int i=0; i<tab_machine.size(); i++){
		ROS_INFO("x :landmarks[%d] = %f", i,tab_machine[i].x);
	}*/
}

bool LocaSubscriber::foundInUnkown(int z)
{
	return (std::find(m_unkownZones.begin(), m_unkownZones.end(), z) != m_unkownZones.end());
}

bool LocaSubscriber::foundInExplored(int z)
{
	return (std::find(m_exploredZones.begin(), m_exploredZones.end(), z) != m_exploredZones.end());
}

bool LocaSubscriber::foundInNotExplored(int z)
{
	return (std::find(m_notExploredZones.begin(), m_notExploredZones.end(), z) != m_notExploredZones.end());
}


void LocaSubscriber::pushToExploredList(int z)
{
	m_exploredZones.push_back(z);
}

void LocaSubscriber::removeFromUnkown(int z)
{
	m_unkownZones.remove(z);
}

void LocaSubscriber::removeFromExplore(int z)
{
	m_exploredZones.remove(z);
}

void LocaSubscriber::removeFromNotExplored(int z)
{
	m_notExploredZones.remove(z);
}
