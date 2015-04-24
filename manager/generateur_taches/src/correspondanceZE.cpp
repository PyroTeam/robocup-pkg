#include "correspondanceZE.h"
#include "comm_msg/ExplorationInfo.h"
#include <vector>

CorrespondanceZE::CorrespondanceZE(){
	m_correspondanceZE_sub = m_nh.subscribe("exploration_info",1000,&comm_msg::ExplorationInfo,this); 	
}

CorrespondanceZE::~CorrespondanceZE(){}

void correspondanceZE::cZECallback(const comm_msg::ExplorationInfo &msg){	
	m_zone_utile.clear();
	for(int i=0;i<zone.size();i++){ 
		int team=0;
		m_nh.getParam("teamColor",team);
		if((team == msg.zone[i].team_color){
			m_zone_utile.push_back(i);
		}
	}
}

std::vector<int> CorrespondanceZE::get_zone_utile(){
	return m_zone_utile;
}
