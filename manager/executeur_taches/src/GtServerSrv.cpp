#include "GtServerSrv.h"

GtServerSrv::GtServerSrv()
{
	ros::NodeHandle n;
    std::string teamColor;
	n.param<int>("robotNumber", m_nbrobot, 0);
	n.param<std::string>("teamColor", teamColor, "cyan");
	m_color = (teamColor == "magenta")? MAGENTA: CYAN;

	m_msg.nb_robot = m_nbrobot;
	m_msg.state = manager_msg::activity::END;
	m_msg.machine_used = manager_msg::activity::NONE;

	m_ei = new ExploInfoSubscriber();
	m_ls = new LocaSubscriber();

	m_activity_pub = n.advertise<manager_msg::activity>("manager/task_exec_state", 50);
}

GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id)
{
	m_id = id;
}

void GtServerSrv::going(geometry_msgs::Pose2D point)
{
   int count = 0, stateOfNavigation;
   do{
		ROS_INFO("Going to point : x: %f; y: %f; theta: %f",point.x,point.y,point.theta);
		NavigationClientAction n_c;
		stateOfNavigation = n_c.goToAPoint(point);
		if(stateOfNavigation == deplacement_msg::MoveToPoseResult::ERROR)
		{
			count ++;
			ROS_WARN("Unable to reach requested point (%f;%f; %f rads). Will try another one"
				, point.x, point.y, point.theta);
			point.x -= 0.2;
			point.y += 0.2;
   		}
	}while (stateOfNavigation == deplacement_msg::MoveToPoseResult::ERROR);
}

geometry_msgs::Pose2D GtServerSrv::calculOutPoint(geometry_msgs::Pose2D pt_actuel, int zone)
{
	geometry_msgs::Pose2D pt_dest, center;
	center.x = m_ls->m_machine[zone - 1].x;
	center.y = m_ls->m_machine[zone - 1].y;
	center.theta = m_ls->m_machine[zone - 1].theta;
	pt_dest.x = 2*center.x - pt_actuel.x;
	pt_dest.y = 2*center.y - pt_actuel.y;
	pt_dest.theta = pt_actuel.theta - M_PI;
	return pt_dest;
}

/* Valentin's function */
void GtServerSrv::getSidePoints(int zone, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2)
{
#define MARGIN_FROM_CENTER 0.75
	geometry_msgs::Pose2D knownMachinePose;
	float dy = 0;
	float dx = 0;

	knownMachinePose.x = m_ls->m_machine[zone - 1].x;
	knownMachinePose.y = m_ls->m_machine[zone - 1].y;
	knownMachinePose.theta = m_ls->m_machine[zone - 1].theta;
	knownMachinePose.theta = fmod(knownMachinePose.theta, M_PI);

	dy = -MARGIN_FROM_CENTER * cos(knownMachinePose.theta);
	dx = MARGIN_FROM_CENTER * sin(knownMachinePose.theta);

	point1.x = knownMachinePose.x - dx;
	point1.y = knownMachinePose.y - dy;
	point1.theta = knownMachinePose.theta - M_PI/2;

	point2.x = knownMachinePose.x + dx;
	point2.y = knownMachinePose.y + dy;
	point2.theta = knownMachinePose.theta + M_PI/2;

#undef MARGIN_FROM_CENTER
}

bool GtServerSrv::knownMachineInZone(int zone)
{
	return m_ls->m_machine[zone - 1].isHere;
}


void GtServerSrv::getNearestPoint(geometry_msgs::Pose2D &pose
	, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2
	, geometry_msgs::Pose2D **targetPointPtr, geometry_msgs::Pose2D **otherPointPtr)
{
	/* TODO: Unfake this function (use pose) */
	*targetPointPtr = &point1;
	*otherPointPtr = &point2;
}

void GtServerSrv::asking(geometry_msgs::Pose2D point)
{
	int count = 0;
	int16_t mid;
	ArTagClienSrv atg;
	FinalApproachingClient fa_c;
	do{
		if(count = 1) {point.y += 1.5; point.theta += M_PI/2;  going(point);}
		else if(count = 2) {point.x -= 2;   point.theta += M_PI/2;  going(point);}
		else if(count = 3) {point.y -= 1.5; point.theta += M_PI/2;  going(point);}
		else if(count = 4) {point.x += 2;   point.theta += M_PI/2;  going(point);}
		else count = 0;
		fa_c.starting(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
		mid = atg.askForId();
		count ++;
	}while(mid == -1);
	m_id = mid;
}

manager_msg::activity GtServerSrv::getActivityMsg()
{
	return m_msg;
}

manager_msg::finalApproachingAction GtServerSrv::getFinalAppAction()
{
	return m_act;
}

void GtServerSrv::interpretationZone()
{
	int zone = this->m_id;
	// Get bottom-right coord of zone
	m_x = 0;
	m_y = 0;
	// Right side
	if(zone>0 && zone<13) 
	{
		m_x = ((zone-1)/4)*2;
		m_y = ((zone-1)%4)*1.5;
		m_x+=2;
	}
	// Left side
	else if (zone<=24) 
	{
		zone -=12;
		m_x = -((zone-1)/4)*2 - 2;
		m_y = ((zone-1)%4)*1.5;
		m_x+=2;
	}
	else 
	{
		ROS_ERROR("There is only 23 zones ");
	}
	m_x+=0.001;
	m_y+=0.001;
}

int GtServerSrv::teamColorOfId(int arTag)
{
	int team_color = 0;
	switch(arTag)
	{
		case  C_CS1_IN    :       team_color = CYAN;          m_name = "C-CS1";       break;

		case  C_CS1_OUT   :       team_color = CYAN;          m_name = "C-CS1";       break;

		case  C_CS2_IN    :       team_color = CYAN;          m_name = "C-CS2";       break;

		case  C_CS2_OUT   :       team_color = CYAN;          m_name = "C-CS2";       break;

		case  C_RS1_IN    :       team_color = CYAN;          m_name = "C-RS1";       break;

		case  C_RS1_OUT   :       team_color = CYAN;          m_name = "C-RS1";       break;

		case  C_RS2_IN    :       team_color = CYAN;          m_name = "C-RS2";       break;

		case  C_RS2_OUT   :       team_color = CYAN;          m_name = "C-RS2";       break;

		case  C_BS_IN     :       team_color = CYAN;          m_name = "C-BS";        break;

		case  C_BS_OUT    :       team_color = CYAN;          m_name = "C-BS";        break;

		case  C_DS_IN     :       team_color = CYAN;          m_name = "C-DS";        break;

		case  C_DS_OUT    :       team_color = CYAN;          m_name = "C-DS";        break;


		case  M_CS1_IN    :       team_color = MAGENTA;       m_name = "M-CS1";       break;

		case  M_CS1_OUT   :       team_color = MAGENTA;       m_name = "M-CS1";       break;

		case  M_CS2_IN    :       team_color = MAGENTA;       m_name = "M-CS2";       break;

		case  M_CS2_OUT   :       team_color = MAGENTA;       m_name = "M-CS2";       break;

		case  M_RS1_IN    :       team_color = MAGENTA;       m_name = "M-RS1";       break;

		case  M_RS1_OUT   :       team_color = MAGENTA;       m_name = "M-RS1";       break;

		case  M_RS2_IN    :       team_color = MAGENTA;       m_name = "M-RS2";       break;

		case  M_RS2_OUT   :       team_color = MAGENTA;       m_name = "M-RS2";       break;

		case  M_BS_IN     :       team_color = MAGENTA;       m_name = "M-BS";        break;

		case  M_BS_OUT    :       team_color = MAGENTA;       m_name = "M-BS";        break;

		case  M_DS_IN     :       team_color = MAGENTA;       m_name = "M-DS";        break;

		case  M_DS_OUT    :       team_color = MAGENTA;       m_name = "M-DS";        break;

		default           :       team_color = -1;            m_name = "";            break;
	}
	return team_color;
}

/**
 * @brief Determine the team of a given zone from ExplorationInfo
 *
 * @param zone target zone
 * @return MAGENTA, CYAN or -1 on error
 */
int GtServerSrv::teamColorOfZone(int zone)
{
#undef CYAN
#undef MAGENTA

	int team_color = -1;

	if (m_ei->m_zones.empty())
	{
		ROS_ERROR("ExplorationInfo zones vector is empty");
		return -1;
	}

	for (std::vector<comm_msg::ExplorationZone>::iterator i = m_ei->m_zones.begin(); i != m_ei->m_zones.end(); ++i)
	{
		if (i->zone == zone)
		{
			switch (i->team_color)
			{
				case comm_msg::ExplorationZone::CYAN:       /* FALLTRHOUGH */
				case comm_msg::ExplorationZone::MAGENTA:
					team_color = i->team_color;
					break;

				default:
					ROS_ERROR("Can't determine the team for zone #%d", zone);
					team_color= -1;
					break;
			}
			break;
		}
	}

	return team_color;
}

bool GtServerSrv::responseToGT(manager_msg::order::Request &req,manager_msg::order::Response &res)
{
	ROS_INFO("Order received");
	ROS_INFO("Request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d"
		, (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
	setId(req.id);
	if (req.number_robot == m_nbrobot)
	{
	  	res.number_order = req.number_order;
	  	res.number_robot = m_nbrobot;
	 	res.id = m_id;
	  	MyElements m;
	  	switch(req.type)   // à rajouter => machine non occupée par un robotino et au départ (on ne sait pas cs1/cs2 et rs1/rs2)
	  	{
		  	case orderRequest::TAKE_BASE:
				m.getBS().take_base(req.parameter,m_nbrobot,req.number_order);
				break;
		  	case orderRequest::PUT_CAP:
				switch(req.parameter)
				{
					case orderRequest::BLACK :
						if(m.getCS1().getBlackCap() != 0)        m.getCS1().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getBlackCap() != 0)   m.getCS2().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
					case orderRequest::GREY :
						if(m.getCS1().getGreyCap() != 0)         m.getCS1().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getGreyCap() != 0)    m.getCS2().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
				}
				break;
		  	case orderRequest::TAKE_CAP:
			   switch(req.parameter)
			   {
					case orderRequest::BLACK :
						if(m.getCS1().getBlackCap() != 0)        m.getCS1().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getBlackCap() != 0)   m.getCS2().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
					case orderRequest::GREY :
						if(m.getCS1().getGreyCap() != 0)         m.getCS1().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getGreyCap() != 0)    m.getCS2().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
				}
				break;
		  	case orderRequest::PUT_RING:
				switch(req.parameter)
				{
					case orderRequest::GREEN :
						if(m.getRS1().getGreenRing() != 0)       m.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getGreenRing() != 0)  m.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::YELLOW :
						if(m.getRS1().getYellowRing() != 0)      m.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getYellowRing() != 0) m.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::BLUE :
						if(m.getRS1().getBlueRing() != 0)        m.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getBlueRing() != 0)   m.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::ORANGE :
						if(m.getRS1().getOrangeRing() != 0)      m.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getOrangeRing() != 0) m.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
				}
				break;
		  	case orderRequest::TAKE_RING:
				switch(req.parameter)
				{
					case orderRequest::GREEN :
						if(m.getRS1().getGreenRing() != 0)       m.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getGreenRing() != 0)  m.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::YELLOW :
						if(m.getRS1().getYellowRing() != 0)      m.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getYellowRing() != 0) m.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::BLUE :
						if(m.getRS1().getBlueRing() != 0)        m.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getBlueRing() != 0)   m.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::ORANGE :
						if(m.getRS1().getOrangeRing() != 0)      m.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getOrangeRing() != 0) m.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
				}
				break;
		  	case orderRequest::BRING_BASE_RS:
				switch(req.parameter)
				{
					case orderRequest::GREEN :
						if(m.getRS1().getGreenRing() != 0)       m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getGreenRing() != 0)  m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::YELLOW :
						if(m.getRS1().getYellowRing() != 0)      m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getYellowRing() != 0) m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::BLUE :
						if(m.getRS1().getBlueRing() != 0)        m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getBlueRing() != 0)   m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
					case orderRequest::ORANGE :
						if(m.getRS1().getOrangeRing() != 0)      m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
						else if(m.getRS2().getOrangeRing() != 0) m.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
						break;
				}
				break;
		  	case orderRequest::DELIVER:
				switch(req.parameter)
				{
					case orderRequest::DS :
						m.getDS().deliverToDS(m_nbrobot,req.number_order);
						break;
					case orderRequest::STOCK :
						int i = 0;
						for(i = 0; i<3; i++)
						{
							if(m.getCS1().getStockage(i) ==0 )
							{
								m.getCS1().stock(i,m_nbrobot,req.number_order,activity::CS1);
								m.getCS1().majStockID(i,1);
								break;
							}
							else if(m.getCS2().getStockage(i+3) ==0 )
							{
								m.getCS2().stock(i+3,m_nbrobot,req.number_order,activity::CS1);
								m.getCS2().majStockID(i+3,1);
								break;
							}
							else
							{
								if(i == 3 ) ROS_ERROR("ERROR: no more place to stock ");
							}
						}
				}
				break;
		  	case orderRequest::UNCAP:
				switch(req.parameter)   // à verifier? chaque CS à des capscat spécifiques
				{
					case orderRequest::BLACK :
						if(m.getCS1().getBlackCap() != 0)        m.getCS1().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getBlackCap() != 0)   m.getCS2().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
					case orderRequest::GREY :
						if(m.getCS1().getGreyCap() != 0)         m.getCS1().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
						else if(m.getCS2().getGreyCap() != 0)    m.getCS2().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
						break;
				}
				break;
		  	case orderRequest::DESTOCK:
				if(req.id >= 0 && req.id < 3)
				{
					m.getCS1().destock(req.id,m_nbrobot,req.number_order,activity::CS1);
				   	m.getCS1().majStockID(req.id, 0);
				}
				else if(req.id >= 3 && req.id < 6)
				{
				   	m.getCS2().destock(req.id,m_nbrobot,req.number_order,activity::CS2);
				   	m.getCS2().majStockID(req.id, 0);
				}
				else
				{
				  	ROS_ERROR("ERROR: req.id is not between 0 and 5 ");
				  	res.accepted =false;
				}
				break;

		  	case orderRequest::DISCOVER:
		  	{
		  		// A partir de zone -> déterminer premier coin zone (plus accessible)
		  		
		  		// Se déplacer au premier coin zone

		  		// A partir detection machine -> voir si machine présente

		  		// Si machine NON présente
		  			// déterminer second coin zone (le plus accessible), avec angle différent du premier
		  			// NB: en cas de mur, pouvoir déterminer un point légèrement décalé

		  			// Se rendre au second coin zone

		  		// A partir detection machine -> voir si machine présente

		  		// Si machine toujours NON présente, abandon

		  		// Si machine présente, déterminer point devant machine
		  		// Se rendre au point devant machine

		  		// Récupérer ArTag ID

		  		// Vérifier si OUTPUT (TODO: à vérifier)

		  		// Si NON output
		  			// Déterminer point devant autre côte de la machine

		  			// Se rendre ou point devant autre côté de la machine

		  			// Récupérer ArTag ID

		  			// Vérifier si OUTPUT, sinon abandonner

		  		// Approche finale, objectif FEU

		  		// Traitement d'image, détection FEU

		  		// Intérprétation type à partir de LightSignal

		  		// Déterminer nom de machine à partir ArTagID ou autres

		  		// Reporter machine








				#if 1 == 0 // Sandra's discover code
				ROS_INFO ("Received discover Order");

				geometry_msgs::Pose2D pt_dest;
				geometry_msgs::Pose2D pt_actuel;

				// TODO: S'assurer que m_id n'est pas utilisé pour tout et n'importe quoi
				// TODO: Affecter req.id à m_id avant d'appeler cette fonction
				interpretationZone();
				pt_dest.x = this->m_x;
				pt_dest.y = this->m_y;
				pt_dest.theta = M_PI/4;

				// Se déplace jusqu'au point
				// TODO: Gérer cas erreur (timeout)
				going(pt_dest);

				ROS_INFO ("I went to the asked point successfully ");

				ROS_INFO("Starting exploring the ARTag ");
				// Fait l'approche finale et récupère l'ID de l'ARTag le plus proche
				// TODO: Renommer asking
				asking(pt_dest);

				// TODO: Attention, teamColorOfId peuple déjà m_name
				// Construit m_name
				int team_color = teamColorOfId(m_id);
				if (team_color == CYAN)         m_name = "C-" + m_name;
				else if(team_color == MAGENTA)  m_name = "M-" + m_name;

				if(team_color != this->m_color)
				{
					ROS_ERROR("Machine isn't for my team ");
					res.accepted = false;
					break;
				}

				/* phase d'exploration */

				ReportingMachineSrvClient rm_c;
				// Ici m_id ne représente plus une zone, mais un ARTag id
				switch(m_id)
				{
					case M_BS_IN  :
					case M_BS_OUT :
					case C_BS_IN  :
					case C_BS_OUT :

						if(m_id == C_BS_OUT || m_id == M_BS_OUT)
					  	{
						  	m_msg = m.getBS().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::BS,req.id);
						  	// TODO: Limiter le nombre de moyen d'appeler une approche finale
						 	m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						  	if(m_ei->m_signals.size() != 0)
						  	{
								m.getBS().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
							}
						}
						else if (m_id == C_BS_IN || m_id == M_BS_IN)
						{
							m_msg = m.getBS().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::BS,req.id);
							pt_actuel = pt_dest;
							pt_dest = calculOutPoint(pt_actuel, req.id);
							going(pt_dest);
							m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
							if(m_ei->m_signals.size() != 0) 
							{
								m.getBS().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
							}
						}
						break;

					case M_RS1_OUT :
					case M_RS1_IN  :
					case C_RS1_IN  :
					case C_RS1_OUT :
					case M_RS2_OUT :
					case M_RS2_IN  :
					case C_RS2_IN  :
					case C_RS2_OUT :

						if(m_id == C_RS1_OUT || m_id == M_RS1_OUT)
						{
							m_msg = m.getRS1().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::RS1,req.id);
							m.getBS().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
							if(m_ei->m_signals.size() != 0) 
							{
								m.getRS1().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								m_name = m_name+"1";
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
							}
						}
						else if(m_id == C_RS2_OUT || m_id == M_RS2_OUT)
						{
							m_msg = m.getRS2().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::RS2,req.id);
							m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
							if(m_ei->m_signals.size() != 0) 
							{
								m.getRS2().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								m_name = m_name+"2";
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
							}
						}
					  	else if(m_id == C_RS1_IN || m_id == M_RS1_IN)
					  	{
						   	m_msg = m.getRS1().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::RS1,req.id);
						   	pt_actuel = pt_dest;
						   	pt_dest = calculOutPoint(pt_actuel, req.id);
						   	going(pt_dest);
						   	m.getRS1().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						   	if(m_ei->m_signals.size() != 0) 
						   	{
							  	m.getRS1().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"1";
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						   	}
					  	}
					  	else if(m_id == C_RS2_IN || m_id == M_RS2_IN)
					  	{
						   	m_msg = m.getRS2().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::RS2,req.id);
						   	pt_actuel = pt_dest;
						   	pt_dest = calculOutPoint(pt_actuel, req.id);
						   	going(pt_dest);
						   	m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						   	if(m_ei->m_signals.size() != 0) 
						   	{
							  	m.getRS2().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"2";
							  	rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						   	}
					  	}
						break;
					case M_CS1_OUT :
				  	case M_CS1_IN  :
				  	case C_CS1_IN  :
				  	case C_CS1_OUT :
				  	case M_CS2_OUT :
				  	case M_CS2_IN  :
				  	case C_CS2_IN  :
				  	case C_CS2_OUT :
					  	if(m_id == C_CS1_OUT || m_id == M_CS1_OUT)
					  	{
						 	m_msg = m.getCS1().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::CS1,req.id);
						  	m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						  	if(m_ei->m_signals.size() != 0) 
						  	{
							  	m.getCS1().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"1";
							  	rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						  	}
					  	}
					  	else if(m_id == C_CS2_OUT || m_id == M_CS2_OUT)
					  	{
						  	m_msg = m.getCS2().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::CS2,req.id);
						  	m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						  	if(m_ei->m_signals.size() != 0) 
						  	{
							  	m.getCS2().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"2";
							  	rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						  	}
					  	}

					  	else if(m_id == C_CS1_IN || m_id == M_CS1_IN)
					  	{
						   	m_msg = m.getCS1().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::CS1,req.id);
						  	pt_actuel = pt_dest;
						   	pt_dest = calculOutPoint(pt_actuel, req.id);
						   	going(pt_dest);
						   	m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						   	if(m_ei->m_signals.size() != 0) 
						   	{
							  	m.getCS1().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"1";
							  	rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
							}
					  	}
					  	else if(m_id == C_CS2_IN || m_id == M_CS2_IN)
					  	{
						   	m_msg = m.getCS2().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::CS2,req.id);
						   	pt_actuel = pt_dest;
						   	pt_dest = calculOutPoint(pt_actuel, req.id);
						   	going(pt_dest);
						   	m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						   	if(m_ei->m_signals.size() != 0) 
						   	{
							  	m.getCS2().readlights(m_ei->lSpec);
							  	m_ei->interpretationFeu();
							  	m_name = m_name+"2";
							  	rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						   	}
					  	}
						break;
					case M_DS_IN  :
					case M_DS_OUT :
					case C_DS_IN  :
					case C_DS_OUT :
					  	if(m_id == C_DS_IN || m_id == M_DS_IN)
					  	{
						   	m_msg = m.getDS().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::DS,req.id);
						   	m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						   	if(m_ei->m_signals.size() != 0) 
						   	{
								m.getDS().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						   	}
					  	}
					  	else if (m_id == C_DS_OUT || m_id == M_DS_OUT)
					  	{
						  	m_msg = m.getDS().msgToGT(m_nbrobot,activity::IN_PROGRESS,activity::DS,req.id);
						  	pt_actuel = pt_dest;
						  	pt_dest = calculOutPoint(pt_actuel, req.id);
						  	going(pt_dest);
						  	//m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
						  	if(m_ei->m_signals.size() != 0) 
						  	{
								m.getDS().readlights(m_ei->lSpec);
								m_ei->interpretationFeu();
								rm_c.reporting(m_name, m_ei->type,/*m_id*/req.id);
						  	}
					  	}
					  	break;
				}
				#endif // Sandra code

				/*----------  Valentin's discover code  ----------*/
				int teamColor = -1;
				int machineSideId = 0;
				// std::string machineLight = "";
				geometry_msgs::Pose2D pose, point1, point2;
				geometry_msgs::Pose2D *targetPointPtr = NULL;
				geometry_msgs::Pose2D *otherPointPtr = NULL;
				ReportingMachineSrvClient reportClient;

				// NOTE (Valentin): Selon moi, le générateur devrait envoyer la zone à explorer dans req.parameter et non dans req.id
				// La ligne ci-dessous est un hack pour que le code de l'executeur soit "conforme" 
				req.parameter = req.id;
				ROS_INFO("Order: DISCOVER, zone %d", req.parameter);
				// Check if valid zone
				teamColor = teamColorOfZone(req.parameter);
				if(teamColor != this->m_color)
				{
					ROS_ERROR("Opposing team or unknown team zone #%d", req.parameter);
					res.accepted = false;
					break;
				}

				// Determine if a MPS is known here
				if ( !knownMachineInZone(req.parameter) )
				{
					ROS_ERROR("No MPS known here. Abort");
					res.accepted = false;
					break;
				}

				// Zone discover approach, i.e. determine where the machine is on zone
				/* NOTE: Not needed if first poc */

				// Do a basic approach on one side
					// Get machine pose
				getSidePoints(req.parameter, point1, point2);
					// Get nearest side
					// Compute target pose (with orientation)
				/* TODO: Use real pose */
				getNearestPoint(pose, point1, point2, &targetPointPtr, &otherPointPtr);
					// Go to pose
				going(*targetPointPtr);

				// Get ArTag id once
				ArTagClienSrv atg;
				machineSideId = atg.askForId();
				std::string machineType = "";
				ROS_INFO("DISCOVER - got artag side id : %d", machineSideId);

				// Detemine if output or input
				/* NOTE: Not needed if first poc */
				/* XXX: Check in rulebook if needed */

				// Determine name
				/* Use more dedicated function */
				teamColorOfId(machineSideId);
				ROS_INFO("DISCOVER - got name from id : %s", m_name.c_str());

				// If output
				/* NOTE: Not needed if first poc */
					// go to input

					// Get ArTag id

					// Comfirm it's and input or abord

				// Do a final approach on light
				/* NOTE: Not needed if first poc */

				// Get light signal
				/* XXX: Use more generic function than a BS machine method */
				m.getBS().readlights(m_ei->m_lSpec);
				ROS_INFO("DISCOVER - got light signal");

				// From light, get type
				m_ei->interpretationFeu();
				ROS_INFO("DISCOVER - got machine type : %s", m_ei->type.c_str());

				// Report
				reportClient.reporting(m_name, m_ei->type, req.parameter);
				ROS_INFO("DISCOVER - reported machine");
			} break;

			default:
				break;
		}
	  	//if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
	  	//else ROS_INFO(" NON DESTOCKAGE ");
	  	m_msg = m.getBS().msgToGT(m_nbrobot,activity::END,activity::NONE,req.id);
	  	res.accepted = true;
	}
	else
	{
		ROS_WARN("Request for another robot");
		res.accepted = false;
	}

	/* VERIFICATIONS */
	ROS_INFO("Requested (reminder): nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d"
		, (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
	ROS_INFO("Response: nb_order=%d, nb_robot=%d", (int)res.number_order, (int)res.number_robot);

return true;
}
