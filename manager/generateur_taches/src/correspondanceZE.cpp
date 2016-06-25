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

int CorrespondanceZE::getBestZone()
{
	return 1;
}
