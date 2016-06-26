#include "correspondanceZE.h"
#include "comm_msg/ExplorationInfo.h"
#include <vector>
#include <string>

CorrespondanceZE::CorrespondanceZE()
: m_unkownZones()
, m_exploredZones()
, m_notExploredZones()
, m_locaSub(m_unkownZones,m_exploredZones,m_notExploredZones)
{
	m_correspondanceZESub = m_nh.subscribe("refBoxComm/ExplorationInfo",1000,&CorrespondanceZE::cZECallback,this);
}

CorrespondanceZE::~CorrespondanceZE(){}

void CorrespondanceZE::cZECallback(const comm_msg::ExplorationInfo &msg){
	m_usefulZone.clear();
	m_unkownZones.clear();
	std::cout << "Taille zone msg = " << msg.zones.size() << std::endl;
	for(int i=0;i<msg.zones.size();i++){
		int team=0;
		std::string teamStr;
		m_nh.getParam("teamColor",teamStr);
		team = (teamStr == "cyan") ? 0 : 1;
		if(team == msg.zones[i].team_color){
			m_usefulZone.push_back(msg.zones[i].zone);
			bool foundInExplored = (std::find(m_exploredZones.begin(), m_exploredZones.end(), msg.zones[i].zone) != m_exploredZones.end());
			bool foundInNotExplored = (std::find(m_notExploredZones.begin(), m_notExploredZones.end(), msg.zones[i].zone) != m_notExploredZones.end());
			if(!foundInExplored && !foundInNotExplored)
			{
				m_unkownZones.push_back(msg.zones[i].zone);
			}
		}
	}
}

std::vector<int> CorrespondanceZE::getUsefulZone(){
	return m_usefulZone;
}

bool sortOnDistance (CorrespondanceZE::s_ZoneDistance s1,CorrespondanceZE::s_ZoneDistance s2) { return (s1.distance < s2.distance); }

int CorrespondanceZE::getBestZone()
{
	common_utils::RobotPoseSubscriber poseSub;
	geometry_msgs::Pose2D actualPose = poseSub.getPose2D();
	std::list<s_ZoneDistance> orderedDistance;

	/* Retourner une zone de la liste m_notExploredZones en priorité */
	for(auto &zone : m_notExploredZones)
	{
		double x,y;
		if(!common_utils::getZoneCenter(zone,x,y))
		{
			ROS_ERROR("HELP ME! I can't get the center of the zone %d", zone);
		}
		else
		{
			geometry_msgs::Pose2D centerZone;
			centerZone.x = x;
			centerZone.y = y;
			s_ZoneDistance tmp_ZD;
			tmp_ZD.distance = geometry_utils::distance(actualPose,centerZone);
			tmp_ZD.zone = zone;
			orderedDistance.push_back(tmp_ZD);
		}
	}

	if(!orderedDistance.empty())
	{
		orderedDistance.sort(sortOnDistance);
		return orderedDistance.front().distance;
	}

	orderedDistance.clear();

	/* Si pas de zone dans m_notExploredZones, on essaie de trouver la zone la plus proche dans la liste m_unkownZones */
	for(auto &zone : m_unkownZones)
	{
		double x,y;
		if(!common_utils::getZoneCenter(zone,x,y))
		{
			ROS_ERROR("HELP ME! I can't get the center of the zone %d", zone);
		}
		else
		{
			geometry_msgs::Pose2D centerZone;
			centerZone.x = x;
			centerZone.y = y;
			s_ZoneDistance tmp_ZD;
			tmp_ZD.distance = geometry_utils::distance(actualPose,centerZone);
			tmp_ZD.zone = zone;
			orderedDistance.push_back(tmp_ZD);
		}
	}

	if(!orderedDistance.empty())
	{
	  orderedDistance.sort(sortOnDistance);
		return orderedDistance.front().distance;
	}

	return -1;
}
