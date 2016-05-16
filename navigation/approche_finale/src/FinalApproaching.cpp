#include "FinalApproaching.h"

#include "Bumperlistener.h"
#include "fa_utils.h"
#include "LaserScan.h"
#include "Point.h"
#include "ArTagFA.h"
#include "OdomFA.h"
#include "Sharps.h"
#include "GameStateFA.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <manager_msg/FinalApproachingAction.h>
#include <approche_finale_msg/plotDataFA.h>

#include <cmath>
#include <vector>
#include <list>

FinalApproaching::~FinalApproaching(void){}

void FinalApproaching::preemptCB()
{
	ROS_INFO("%s: Preempted", actionName.c_str());
	// set the action state to preempted
	as.setPreempted();
}

void FinalApproaching::executeCB(const manager_msg::FinalApproachingGoalConstPtr &goal)
{
	ros::Rate loopRate(100);
	m_pubMvt = nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel",1000);
	m_plot  = nh.advertise<approche_finale_msg::plotDataFA>("rosplot/plotDataFA",1000);
	approche_finale_msg::plotDataFA plotData;

	// General initialization
	bool success = true;
	feedback.percent_complete = 0;
	geometry_msgs::Twist msgTwist;
	bool usefulInfo = false;

	// BumperListener
	BumperListener bp;

	// Odom
	OdomFA odom;

	float initOrientation = odom.getOrientationZ();
	bool initOdom = false;

	// ARTag
	ArTagFA at;
	int k=-1;
	int phase = 0;
	int avancementArTag = 0;
	std::vector<int> id;
	std::vector<float> px = at.getPositionX();
	std::vector<float> pz = at.getPositionZ();
	std::vector<float> oz = at.getOrientationZ();
	std::vector<float> arTagDistance = at.getDistance();

	// Sharps
	Sharps sharps;
	bool obstacle = false;
	std::vector<bool> allObstacles = sharps.getObstacle();

	// GameState
	GameStateFA gameState;
	int teamColor;
	nh.param<int>("teamColor",teamColor,0);

	// LaserScan
	LaserScan ls;
	int a=0, b=0, c=0, cpt=0;
	float positionY=0, gradient=0, ortho=0, moyY=0, moyO=0;
    int j = 0;
    std::list<float> listPositionY, listOrtho;


    //to get goal action   		  
	ROS_INFO("%s: Executing, creating FinalApproaching sequence of type %i with side %i and parameter %i", actionName.c_str(), goal->type, goal->side,goal->parameter);
	m_type = goal->type;
	m_side = goal->side;
	m_parameter = goal->parameter;
	//at least one callback for odometry and gamestate
	while(ros::ok() && !bp.getState() && !usefulInfo)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}
		if(odom.getTurn() /*&& gameState.getAlready()*/)
		{
			usefulInfo = true;
		}
		loopRate.sleep();
	}
	//loop to find a correct ARTag
	std::vector<int> allPossibleId = idWanted(teamColor,0/*gameState.getPhase()*/);
	while(ros::ok() && !bp.getState() && k==-1 && phase!=3)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}		
		if(at.getFoundId())
		{	
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
		}
		if(phase == 0)
		{
			initOrientation = odom.getOrientationZ();
			phase++;
		}
		msgTwist.angular.z = cameraScanVelocity(phase);
		float newOrientation = odom.getOrientationZ()-initOrientation;
		phase = phaseDependingOnOrientation(newOrientation,phase);
		m_pubMvt.publish(msgTwist);
		loopRate.sleep();
	}
	//lock loop with ARTag camera 
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==0 && obstacle==false)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}		
		//if at least one ARTag found	
		if(at.getFoundId())
		{
			px=at.getPositionX();
			pz=at.getPositionZ();
			oz=at.getOrientationZ();
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
			ROS_DEBUG("taille de px: %d de pz: %d de oz: %d et valeur de k: %d",
			          (int)px.size(),(int)pz.size(),(int)oz.size(),k);
			avancementArTag=FinalApproaching::asservissementCamera(px,pz,oz,k);
		}
		allObstacles = sharps.getObstacle();
		obstacle = false;//obstacleDetection(allObstacles, k, oz, pz);
		loopRate.sleep();
	}
	ROS_INFO("Waiting a complete laserscan");
	//lock loop with laserscan
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==1 && c!=2 && obstacle==false)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}	
		// if the scan is complete
		if(ls.getRanges().size() == 513)
		{
			std::vector<float> ranges = ls.getRanges();
			float angleMin = ls.getAngleMin();
			double angleInc = ls.getAngleInc();
			float rangeMin = ls.getRangeMin();
			float rangeMax = ls.getRangeMax();
			std::list<std::vector<Point> > tabPoints = objectsConstruction(ranges, angleMin, angleInc, rangeMin, rangeMax);
			std::vector<Segment> tabSegments = segmentsConstruction(tabPoints, ranges, angleMin, angleInc);
			//at least one segment found
			if(tabSegments.size()>0)
			{
				Segment seg = tabSegments[0];
				ROS_DEBUG("min seg: %d max seg %d",seg.getMinRanges(),seg.getMaxRanges());
				//if no error about the laserscan data
				if(seg.getMinRanges()>=0 && seg.getMinRanges()<=513 && seg.getMaxRanges()>=0 && seg.getMaxRanges()<=513)
				{
					gradient = seg.getGradient();
					ROS_DEBUG("gradient du segment le plus proche : %f",gradient);
					positionY = positionYLaser(seg, ranges, angleMin, angleInc);
					ROS_INFO("l objet se trouve a environ %f m du bord",positionY);
					ortho = distanceOrtho(seg, ranges, angleMin, angleInc);
					ROS_DEBUG("distance orthogonale laser-machine : %f",ortho);
					
					//to do an average 
					if(j<1)
					{
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						j++;
					}
					else
					{
						listPositionY.pop_front();
						listOrtho.pop_front();
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						moyY = moy(listPositionY);
						moyO = moy(listOrtho);
						//to be in front of the machine
						a = asservissementAngle(plotData,m_pubMvt,gradient);	
						int min = seg.getMinRanges();
						int max = seg.getMaxRanges();
						Point gauche(ranges[max],angleMin+(double)min*angleInc);
						Point droite(ranges[min],angleMin+(double)max*angleInc);
						plotData.XLeft = gauche.getX();
						plotData.YLeft = gauche.getY();
						plotData.XRight = droite.getX();
						plotData.YRight = droite.getY();
						if((a == 1) && (cpt < 200))
						{
							//to move on the Y axis of the robot
							b = asservissementPositionY(plotData,m_pubMvt,moyY,objectifY(),gauche.getY(),droite.getY());
							ROS_INFO("positionY: %f",positionY);
							//listPositionY.clear();
							if(b==1)
							{
								cpt++;
							}
						}	
						if(a == 1 && cpt >= 200)
						{
							//to move on the X axis of the robot
							c = asservissementPositionX(plotData,m_pubMvt,ortho,objectifX());	
						}
					}	
				}
				m_plot.publish(plotData);
				feedback.percent_complete = avancement(a,b,c);
				as.publishFeedback(feedback);
				ros::spinOnce();
				ROS_DEBUG("etat du bumper: %d ",bp.getState());
			}
			loopRate.sleep();
		}
	}
	//reinitialisation
	geometry_msgs::Twist stop;
	a=0;
	b=0;
	c=0;
	cpt=0;
	j=0;
	avancementArTag=0;
	phase=0;
	listPositionY.clear();
	listOrtho.clear();
	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.angular.z = 0;
	m_pubMvt.publish(stop);
	//if any problem
	if(bp.getState() || phase==3 || obstacle==true)
	{
		if(bp.getState())
		{
			ROS_WARN("OBSTACLE RENCONTRE\n");
			result.state = 1;
		}
		if(phase == 3)
		{
			ROS_WARN("UN BALAYAGE COMPLET SANS ARTAG CORRECT\n");
			result.state = 3;
		}
		if(obstacle==true)
		{
			ROS_WARN("OBSTACLE TROP PROCHE\n");
			result.state = 2;
		}
		success=false;	
	}
	if(success)
	{
		result.success = true;
		result.state = 0;
		ROS_INFO("%s: Succeeded", actionName.c_str());
		as.setSucceeded(result);
	}
	else
	{
		result.success = false;
		ROS_INFO("%s: Aborted",actionName.c_str());
		as.setAborted(result);
	}
}


int FinalApproaching::avancement(int a, int b, int c)
{
	int tmp=-20;
	if(c==2)
	{
		tmp = 100;
	}
	else
	{
		if(b == 1 && (c == 0 || c==1))
		{
			tmp = 67;
		}	
		else
		{
			if(a==1 && b==0 && (c == 0 || c==1))
			{
			tmp = 33;
			}
			else
			{
				tmp = 0;
			}
		}
	}
	return tmp;	
}


float FinalApproaching::objectifY()
{
	float tmp=0;
	switch(m_parameter)
	{
		case manager_msg::FinalApproachingGoal::S1 :
			tmp = 0.28; break;
		case manager_msg::FinalApproachingGoal::S2 :
			tmp = 0.175; break;
		case manager_msg::FinalApproachingGoal::S3 :
			tmp = 0.08; break;
		case manager_msg::FinalApproachingGoal::LANE_RS :
				tmp = 0.09; break;
		case manager_msg::FinalApproachingGoal::LIGHT :
			tmp = 0.35; break;
		case manager_msg::FinalApproachingGoal::CONVEYOR :
			switch(m_side)
			{
				case manager_msg::FinalApproachingGoal::IN :
					tmp = 0.37; break;
				case manager_msg::FinalApproachingGoal::OUT :
					tmp = 0.315; break;
			}
		default: break;
	}
	return tmp;
}

float FinalApproaching::objectifX()
{
	float tmp = 0;
	if(m_parameter == manager_msg::FinalApproachingGoal::LIGHT)
	{
		tmp = 0.35;
	}
	else
	{
		tmp = 0.16;
	}
	return tmp;
}

std::list<std::vector<Point> > FinalApproaching::objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc, float rangeMin, float rangeMax)
{
	std::list<std::vector<Point> > tabPoints;
	Point p0(ranges[0],angleMin);
	std::vector<Point> tabP0;
	tabP0.push_back(p0);
	tabPoints.push_back(tabP0);
	std::list<std::vector<Point> >::iterator it = tabPoints.begin();
	for (int i=1; i<ranges.size(); i++)
	{
		if((ranges[i]>rangeMin) && (ranges[i]<rangeMax))
		{
			Point p(ranges[i],angleMin + (float)i*angleInc);
			//if nearby points (less than 5 cm)
			if(std::abs(ranges[i] - ranges[i-1]) < 0.05)
			{
				it->push_back(p);
			}
			else
			{
				//new object
				std::vector<Point> tabP;
				tabP.push_back(p);
				tabPoints.push_back(tabP);
				it++;
			}
		}
	}
	ROS_DEBUG("nombre d elements de tabPoints: %d",(int)tabPoints.size());
	int j=0,min=0;
	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		ROS_DEBUG("nombre de points de l objet %d = %d",j,(int)it->size());
		j++;
		min = min + it->size();
	}
	return tabPoints;
}


std::vector<Segment> FinalApproaching::segmentsConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc)
{
	std::vector<Segment> tabSegments;
	std::list<std::vector<Point> >::iterator it;
	int min=0,i=0;
	for(it = tabPoints.begin();it != tabPoints.end(); it++)
	{
		float d=0;
		Point pmin(0,0);
		Point pmax(0,0);
		if(it->size()>1)
		{
			ROS_ERROR_COND(ranges.size() <= min || min < 0, "OOPS, Bad index vector access, after line %d in file %s", __LINE__, __FILE__);
			pmin.setR(ranges[min]);
			pmin.setPhi(angleMin+(double)min*angleInc);
			ROS_ERROR_COND(ranges.size() <= min+it->size()-1 || min+it->size()-1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
			pmax.setR(ranges[min+it->size()-1]);
			pmax.setPhi(angleMin+angleInc*(double)(min+it->size()-1));
			d = objectLength(i,min+it->size()-1,tabPoints,ranges,angleMin,angleInc);
		}
		//if object length around 70 cm
		if(d>0.65 && d<0.75)
		{
			//construction of segment
			Segment segm(pmin,pmax,min,it->size() + min -1);
			ROS_DEBUG("distance objet %d du laser = %f",i,segm.distanceLaserSegment(ranges));
			ROS_DEBUG("distance ortho de l objet %d: %f",i,distanceOrtho(segm,ranges,angleMin,angleInc));
			geometry_msgs::Pose2D p=segm.linearRegression(*it);
			geometry_msgs::Point orthoMin = orthoProjection(pmin,p);
			geometry_msgs::Point orthoMax = orthoProjection(pmax,p);
			ROS_INFO("pmin.x: %f pmin.y: %f pmax.x: %f pmax.y: %f",pmin.getX(),pmin.getY(),pmax.getX(),pmax.getY());
			ROS_INFO("omin.x: %f omin.y: %f omax.x: %f omax.y: %f",orthoMin.x,orthoMin.y,orthoMax.x,orthoMax.y); 
			segm.setMinPoint(orthoMin);
			segm.setMaxPoint(orthoMax);
			ROS_DEBUG("taille du segment = %f",d); 
			segm.setDistance(d);
			tabSegments.push_back(segm);
		}
		min = min + it->size();
		i++;
	}
	Segment s;
	ROS_DEBUG("nombre de segment d environ 70 cm : %d",(int)tabSegments.size());
	//only the nearest segment is kept
	if(tabSegments.size()>1)
	{
		s=tabSegments[nearestSegment(tabSegments,ranges)];
		tabSegments.clear();
		tabSegments.push_back(s);
	}
	return tabSegments;
}

float FinalApproaching::objectLength(int i, int j,std::list<std::vector<Point> > tabPoints,std::vector<float> ranges, float angleMin, double angleInc)
{
	std::list<std::vector<Point> >::iterator it = tabPoints.begin();
	int compteur=0;
	int cpt2 =0;
	while(compteur!=i)
	{
		compteur++;
		cpt2 = cpt2 + it->size(); 
		it++;
	}
	if(it!=tabPoints.end() && i!=j)
	{
		std::vector<Point> tab = *it;
		Point pmin(ranges[i],angleMin+(double)i*angleInc);
		Point pmax(ranges[j],angleMin+(double)j*angleInc);
		return distance2points(tab[0],tab[tab.size()-1]);
	}
	else
	{
		return 0;
	}
}

int FinalApproaching::nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges)
{
	int nearest=0;
	std::vector<Segment>::iterator it;
	for(int i=0; i<tabSegments.size(); i++)
	{
		ROS_ERROR_COND(tabSegments.size() <= i || i < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		ROS_ERROR_COND(ranges.size() != 513, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		if(tabSegments[i].distanceLaserSegment(ranges) < tabSegments[nearest].distanceLaserSegment(ranges))
		{
			nearest = i;
		}
	}
	return nearest;
}

//condition préalable: la machine est a 90° du laser
float FinalApproaching::distanceOrtho(Segment s,std::vector<float> ranges,float angleMin, double angleInc)
{
	float ortho=0.0;
	int min = s.getMinRanges();
	int max = s.getMaxRanges();
	ROS_ERROR_COND(ranges.size() <= max || max < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
	Point gauche(ranges[max],angleMin+(double)max*angleInc);
	ROS_ERROR_COND(ranges.size() <= min || min < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
	Point droite(ranges[min],angleMin+(double)min*angleInc);
	//si le laser se trouve entre les deux points extremes du segment
	//si yg et yd sont de signes différents
	ROS_DEBUG("xg=%f xd=%f",gauche.getX(),droite.getX());
	ROS_DEBUG("yg=%f yd=%f",gauche.getY(),droite.getY());
	//if the laser is between the two extremities of the machine
	if(gauche.getY()*droite.getY()<0)
	{
		int tmp = min;
		for(int i=min;i<=max;i++)
		{
			if(ranges[tmp] > ranges[i])
			{
				tmp = i;
			}
		}
		ortho = ranges[tmp];
	}
	else
	{
		float orthoMin = ranges[min]*cos(angleMin+(double)min*angleInc);
		float orthoMax = ranges[max]*cos(angleMin+(double)max*angleInc);
		ortho = (orthoMin+orthoMax)/(float)2;
		ROS_INFO("orthoMin: %f orthoMax: %f",orthoMin,orthoMax);
	}
	return ortho;	
}

float FinalApproaching::positionYLaser(Segment s,std::vector<float> ranges, float angleMin, double angleInc)
{
	/*
	int tmp = 0;
	int min = s.getMinRanges();
	int i = 0;
	int max=s.getMaxRanges();
	for(int i=min; i<max; i++)
	{
		if(ranges[tmp] > ranges[i])
		{
			tmp = i;
		}
	}
	float t = s.getDistance();
	float d = distanceOrtho(s,ranges,angleMin,angleInc);
	//to get the real extremity
	while(std::abs(ranges[tmp-i]-ranges[tmp-1-i])<0.03)
	{
		ROS_ERROR_COND(ranges.size() <= tmp-i || tmp-i < 0 || ranges.size() <= tmp-i-1 || tmp-i-1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		i++;
	}
	float right = sqrt(ranges[tmp-i+1]*ranges[tmp-i+1] - d*d);	
	i=0;
	//to get the other real extremity
	while(std::abs(ranges[tmp+i]-ranges[tmp+i+1])<0.03)
	{
		ROS_ERROR_COND(ranges.size() <= tmp+i || tmp+i < 0 || ranges.size() <= tmp+i+1 || tmp+i+1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		i++;
	}
	float left = sqrt(ranges[tmp+i-1]*ranges[tmp+i-1] -d*d);
	ROS_DEBUG("taille: %f ortho: %f",t,d);
	ROS_DEBUG("right: %f t-left: %f",left,t-right);
	//do an average
	return (right+t-left)/(float)2;
	*/
	//float leftr = ranges[tmp+i-1];
	//float leftphi = (float)(tmp+i-1)*(angleMin+(float)(tmp+i-1)*angleInc);
	geometry_msgs::Point right = s.getMinPoint();
	geometry_msgs::Point left = s.getMaxPoint();
	float segmentSize = sqrt((left.x-right.x)*(left.x-right.x)+(left.y-right.y)*(left.y-right.y));
	ROS_INFO("left.x: %f left.y: %f right.x: %f right.y: %f segmentSize: %f",left.x,left.y,right.x,right.y,segmentSize);
	return right.y;
	//return (left.y-segmentSize+right.y)/(float)2;
}

std::vector<int> FinalApproaching::idWanted(int team,int phase)
{
	std::vector<int> tabId;
	//exploration phase
	if(phase==0)
	{
		switch(team)
		{
			//cyan
			case 0:
				tabId.push_back(2);//CS1 Out
				tabId.push_back(18);//CS2 Out
				tabId.push_back(34);//RS1 Out
				tabId.push_back(178);//RS2 Out
				tabId.push_back(66);//BS Out
				tabId.push_back(81);//DS In
				break;
			//magenta
			case 1:	
				tabId.push_back(98);//CS1 Out
				tabId.push_back(114);//CS2 Out
				tabId.push_back(130);//RS1 Out
				tabId.push_back(146);//RS2 Out
				tabId.push_back(162);//BS Out
				tabId.push_back(49);//DS In
				break;
		}
	}
	//production phase
	else
	{
		switch(team)
		{
			//cyan
			case 0:
			switch(m_type)
			{
				//BS
				case 0:
					tabId.push_back(65);
					break;
				//RS
				case 1:
					tabId.push_back(33);
					tabId.push_back(177);
					break;
				//CS
				case 2:
					tabId.push_back(1);
					tabId.push_back(17);
					break;
				//DS
				case 3:
					tabId.push_back(81);
					break;					
			}
			//magenta
			case 1:
			switch(m_type)
			{
				case 0:
					tabId.push_back(161);
					break;
				case 1:
					tabId.push_back(129);
					tabId.push_back(145);
					break;
				case 2:
					tabId.push_back(97);
					tabId.push_back(113);
					break;
				case 3:
					tabId.push_back(49);
					break;
			}
		}
		//out
		if(m_type==101)
		{
			for(int i=0; i<tabId.size(); i++)
			{
				tabId[i]++;
			}
		}
	}
	return tabId;
}

int FinalApproaching::correspondingId(std::vector<int> allPossibleId,std::vector<int> arTagId,std::vector<float> arTagDistance)
{
	int correspondingId = -1;
	std::vector<int> ids;
	ids.clear();
	if(!arTagId.empty())
	{
		//loop to get the good ARTags
		for(int i=0; i<allPossibleId.size(); i++)
		{
			for(int k=0; k<arTagId.size(); k++)
			{
				if(arTagId[k]==allPossibleId[i])
				{
					ids.push_back(k);
				}
			}
		}
		if(!ids.empty() && !arTagDistance.empty())
		{
			int tmp = ids[0];
			ROS_DEBUG("taille de ids: %d de arTagDistance: %d",(int)ids.size(),(int)arTagDistance.size());
			//loop to keep the nearest ARTag
			for(int j=0; j<ids.size(); j++)
			{
				ROS_DEBUG("ids[j]: %d tmp: %d",ids[j],tmp);
				if(arTagDistance[ids[j]]<arTagDistance[tmp])
				{
					tmp = ids[j];
				}
			}
			correspondingId = tmp;
		}
	}
	return correspondingId;
}

int FinalApproaching::asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz,int k)
{
	int avancementArTag=0;
	geometry_msgs::Twist msgTwist;
	if(!px.empty() && !pz.empty() && !oz.empty())
	{
		if(std::abs(px[k]) < 0.005)
        	{
			msgTwist.linear.y = 0;
		}
		else
		{
			msgTwist.linear.y = -0.75*px[k];
		}
		if(std::abs(pz[k]-0.50) < 0.01)
		{
			msgTwist.linear.x = 0;
		}
		else
		{
			msgTwist.linear.x = 0.25*(pz[k]-0.50);
		}
		if(std::abs(oz[k]) < 0.01)
		{
			msgTwist.angular.z = 0;
		}
		else
		{
			msgTwist.angular.z = 0.125*oz[k];
		}
		if(msgTwist.linear.x==0 && msgTwist.linear.y==0 && msgTwist.angular.z==0)
		{
			avancementArTag=1;
		}
	}
	else
	{
		msgTwist.linear.x=0;
		msgTwist.linear.y=0;
		msgTwist.angular.z=0;
	}
	m_pubMvt.publish(msgTwist);
	return avancementArTag;
}

float FinalApproaching::cameraScanVelocity(int phase)
{
	float tmp;
	switch(phase)
	{
		case 1:
			//turn on the left
			tmp = 0.25; 
			break;
		case 2:
			//turn on the right
			tmp = -0.25; 
			break;
		case 3: 
			tmp = 0; 
			break;
	}
	return tmp;
}

int FinalApproaching::phaseDependingOnOrientation(float newOrientation, int phase)
{
	//to have newOrientation between -PI and PI
	if(newOrientation<-M_PI)
	{
		newOrientation=newOrientation+2*M_PI;
	}
	if(newOrientation>M_PI)
	{
		newOrientation=newOrientation-2*M_PI;
	}
	//if robot have finished turning left
	if(newOrientation>M_PI_2 && phase==1)
	{
		phase = 2;
	}
	//if robot have finished turning right
	if(newOrientation<-M_PI_2 && phase==2)
	{
		phase = 3;
	}
	return phase;
}

bool FinalApproaching::obstacleDetection(std::vector<bool> allObstacles,int k,std::vector<float> oz,std::vector<float> pz)
{
	bool obstacle = false;
	if(k!=-1 && oz.size()>0 && pz.size()>0)
	{	
		//probably wheels of the machine
		if(std::abs(oz[k])>0.45)
		{
			allObstacles[0] = false;
			allObstacles[1] = false;
			allObstacles[8] = false;
		}
		if(pz[k]<0.55)
		{
			allObstacles[0] = false;
		}
	}
	for(int i=0;i<allObstacles.size(); i++)
	{
		if(allObstacles[i]==true)
		{
			obstacle = true;
		}
	}
	return obstacle;
}
