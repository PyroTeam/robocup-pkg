#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <manager_msg/finalApproachingAction.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <cmath>
#include <vector>
#include <list>

#include "finalApproaching.h"
#include "bumperlistener.h"
#include "fa_utils.h"
#include "laserScan.h"
#include "Point.h"
#include "arTagFA.h"
#include "odomFA.h"


finalApproaching::~finalApproaching(void){}

void finalApproaching::executeCB(const manager_msg::finalApproachingGoalConstPtr &goal)
{
	// helper variables
	ros::Rate loopRate(100);
	m_pubMvt = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	//m_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1000);
	bool success = true;
	BumperListener bp;
	OdomFA odom;
	ArTagFA at;
	loopRate.sleep();
	feedback.percent_complete = 0;
	// check that preempt has not been requested by the client
	if (as.isPreemptRequested() || !ros::ok())
	{
		ROS_INFO("%s: Preempted", actionName.c_str());
		// set the action state to preempted
		as.setPreempted();
		success = false;
		//break;
	}
	// publish info to the console for the user
	ROS_INFO("%s: Executing, creating finalApproaching sequence of type %i with side %i and parameter %i", actionName.c_str(), goal->type, goal->side,goal->parameter);
	m_type = goal->type;
	m_side = goal->side;
	m_parameter = goal->parameter;
	float initOrientation = odom.getOrientationZ();
	bool initOdom = false;
	std::vector<int> id;
	std::vector<float> px = at.getPositionX();
	std::vector<float> pz = at.getPositionZ();
	std::vector<float> oz = at.getOrientationZ();
	std::vector<float> arTagDistance = at.getDistance();
	int a=0, b=0, c=0, cpt=0;
	int avancementArTag = 0;
	float positionY=0, gradient=0, ortho=0, moyY=0, moyO=0;
    	int j = 0;
    	std::list<float> listPositionY, listOrtho;   		  
	laserScan ls;
	loopRate.sleep();
	/*visualization::Marker msg_marker;
	msg_marker.header.frame.id = "/laser_link";
	msg_marker.header.stamp = ros::Time::now();
	msg_marker.ns = "visualization_segments";
	msg_marker.action = visualization_msgs::Marjer::ADD;
	msg_marker.pose.orientation.w = 1.0;
	msg_marker.id = 2;
	msg_marker.type = visualization_msgs::Marker::LINE_LIST;
	msg_marker.scale.x = 0.1;
	msg_marker.color.r = 1.0;
	msg_marker.color.a = 1.0;
	geometry_msgs::Point p;
	msg_marker.points.push_back(p);
	msg_marker.points.push_back(p);*/
	std::vector<int> allPossibleId = idWanted(0,0);
	int reverse = 0;
	int k=-1;
	int phase = 0;
	geometry_msgs::Twist msgTwist;
	while(ros::ok() && !bp.getState() && !odom.getTurn())
	{
	}
	while(ros::ok() && !bp.getState() && k==-1 && phase!=3)
	{
		if(at.getFoundId())
		{	
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
		}
		ROS_INFO("phase: %d",phase);
		switch(phase)
		{
			case 0:
				initOrientation = odom.getOrientationZ();
				phase++;
				break;
			case 1:
				msgTwist.angular.z = 0.25; 
				break;
			case 2:
				msgTwist.angular.z = -0.25; 
				break;
			case 3: 
				msgTwist.angular.z = 0; 
				break;
		}
		float newOrientation = odom.getOrientationZ()-initOrientation;
		if(newOrientation<-1)
		{
			newOrientation=newOrientation+2;
		}
		if(newOrientation>1)
		{
			newOrientation=newOrientation-2;
		}
		ROS_INFO("initOrientation: %f odom.getOrientationZ(): %f",initOrientation,odom.getOrientationZ());
		ROS_INFO("newOrientation: %f",newOrientation);
		if(newOrientation>0.5 && phase==1)
		{
			phase = 2;
		}
		if(newOrientation<-0.5 && phase==2)
		{
			phase = 3;
		}
		/*if(initOdom==false && initOrientation!=0)
		{
			initOrientation = odom.getOrientationZ();
			initOdom = true;
		}
		float newOrientation = odom.getOrientationZ();
		if(initOrientation*newOrientation<0)
                {
                        initOrientation=newOrientation;
                        reverse++;
                }
		if(initOrientation==0)
                {
                        initOrientation = newOrientation;
                }
		ROS_INFO("initOrientation: %f",initOrientation);
		msgTwist.angular.z = 0.25;*/
		m_pubMvt.publish(msgTwist);
	}
	ROS_INFO("k: %d reverse: %d",k,reverse);
	msgTwist.angular.z = 0;
	m_pubMvt.publish(msgTwist);
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==0)
	{
		if(at.getFoundId())
		{
			px=at.getPositionX();
			pz=at.getPositionZ();
			oz=at.getOrientationZ();
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
			//ROS_INFO("taille de px: %d de pz: %d de oz: %d et valeur de k: %d",(int)px.size(),(int)pz.size(),(int)oz.size(),k);
			avancementArTag=finalApproaching::asservissementCamera(px,pz,oz,k);
		}
	}
	ROS_INFO("avancementArTag: %d",avancementArTag);
	ROS_INFO("Waiting a complete laserscan");
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==1 && c!=2)
	{
		if(ls.getRanges().size() == 513)
		{
			std::vector<float> ranges = ls.getRanges();
			float angleMin = ls.getAngleMin();
			double angleInc = ls.getAngleInc();
			float rangeMin = ls.getRangeMin();
			float rangeMax = ls.getRangeMax();
			std::list<std::vector<Point> > tabPoints = objectsConstruction(ranges, angleMin, angleInc, rangeMin, rangeMax);
			std::vector<Segment> tabSegments = segmentsConstruction(tabPoints, ranges, angleMin, angleInc);
			if(tabSegments.size()>0)
			{
				Segment seg = tabSegments[0];
				//ROS_INFO("nearestSegment: %d",nearestSegment(tabSegments,ranges));
				ROS_INFO("min seg: %d max seg %d",seg.getMinRanges(),seg.getMaxRanges());
				if(seg.getMinRanges()>=0 && seg.getMinRanges()<=513 && seg.getMaxRanges()>=0 && seg.getMaxRanges()<=513)
				{
					/*msg_marker.points[0].x;
					msg_marker.points[0].y;
					msg_marker.points[1].x;
					msg_marker.points[1].y;
					m_marker_pub.publish(msg_marker);*/
					//std::vector<Segment> tabSeg = ls.getTabSegments();
					gradient = seg.getGradient();
					ROS_INFO("gradient du segment le plus proche : %f",gradient);
					positionY = positionYLaser(seg, ranges, angleMin, angleInc);
					ROS_INFO("l objet se trouve a environ %f m du bord",positionY);
					ortho = distanceOrtho(seg, ranges, angleMin, angleInc);
					ROS_INFO("distance orthogonale laser-machine : %f",ortho);
					if(j<2)
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
						/*listPositionY.clear();
						listOrtho.clear();
						j=0;*/
						a = asservissementAngle(m_pubMvt,gradient);	
						int min = seg.getMinRanges();
						int max = seg.getMaxRanges();
						Point gauche(ranges[max],angleMin+(double)min*angleInc);
						Point droite(ranges[min],angleMin+(double)max*angleInc);
						if(gauche.getY()*droite.getY()<0)
						{
							ROS_INFO("\tlaser au CENTRE de la machine");
						}
						else
						{
							if(gauche.getY()>0)
							{
								ROS_INFO("\tlaser a DROITE de la machine");
							}
							else
							{
								ROS_INFO("\tlaser a GAUCHE de la machine");
							}
						}
						if((a == 1) && (cpt < 200))
						{
							b = asservissementPositionY(m_pubMvt,moyY,objectifY(),gauche.getY(),droite.getY());
							if(b==1)
							{
								cpt++;
							}
						}	
						if(a == 1 && cpt >= 200)
						{
							c = asservissementPositionX(m_pubMvt,ortho,objectifX());	
						}
					}	
				}
				feedback.percent_complete = avancement(a,b,c);
				as.publishFeedback(feedback);
				ros::spinOnce();
				ROS_INFO("etat du bumper: %d ",bp.getState());
				loopRate.sleep();
			}
		}
	}
	if(bp.getState() || phase==3)
	{
		if(bp.getState())
		{
			ROS_WARN("OBSTACLE RENCONTRE\n");
		}
		if(phase == 3)
		{
			ROS_WARN("UN TOUR COMPLET SANS ARTAG CORRECT\n");
		}
		success=false;
		geometry_msgs::Twist stop;
		a=0;
		b=0;
		c=0;
		cpt=0;
		j=0;
		avancementArTag=0;
		reverse=0;
		phase=0;
		listPositionY.clear();
		listOrtho.clear();
		stop.linear.x = 0;
		stop.linear.y = 0;
		stop.angular.z = 0;
		m_pubMvt.publish(stop);
	}
	if(success)
	{
		result.success = true;
		ROS_INFO("%s: Succeeded", actionName.c_str());
		// set the action state to succeeded
		a=0;
		b=0;
		c=0;
		cpt=0;
		avancementArTag=0;
		reverse=0;
		phase=0;
		listPositionY.clear();
		listOrtho.clear();
		j=0;
		as.setSucceeded(result);
	}
}


int finalApproaching::avancement(int a, int b, int c)
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


float finalApproaching::objectifY()
{
	float tmp=0;
	switch(m_parameter)
	{
		case manager_msg::finalApproachingGoal::S1 :
			tmp = 0.28; break;
		case manager_msg::finalApproachingGoal::S2 :
			tmp = 0.175; break;
		case manager_msg::finalApproachingGoal::S3 :
			tmp = 0.08; break;
		case manager_msg::finalApproachingGoal::LANE_RS :
				tmp = 0.09; break;
		case manager_msg::finalApproachingGoal::LIGHT :
			tmp = 0.35; break;
		case manager_msg::finalApproachingGoal::CONVEYOR :
			switch(m_side)
			{
				case manager_msg::finalApproachingGoal::IN :
					tmp = 0.37; break;
				case manager_msg::finalApproachingGoal::OUT :
					tmp = 0.32; break;
			}
		default: break;
	}
	return tmp;
}

float finalApproaching::objectifX()
{
	float tmp = 0;
	if(m_parameter == manager_msg::finalApproachingGoal::LIGHT)
	{
		tmp = 0.35;
	}
	else
	{
		tmp = 0.16;
	}
	return tmp;
}

std::list<std::vector<Point> > finalApproaching::objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc, float rangeMin, float rangeMax)
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
			if(std::abs(ranges[i] - ranges[i-1]) < 0.05)
			{
				it->push_back(p);
			}
			else
			{
				std::vector<Point> tabP;
				tabP.push_back(p);
				tabPoints.push_back(tabP);
				it++;
			}
		}
	}
	ROS_INFO("nombre d elements de tabPoints: %d",(int)tabPoints.size());
	int j=0,min=0;
	for(it=tabPoints.begin();it!=tabPoints.end();it++)
	{
		ROS_INFO("nombre de points de l objet %d = %d",j,(int)it->size());
		j++;
		min = min + it->size();
	}
	return tabPoints;
}


std::vector<Segment> finalApproaching::segmentsConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc)
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
			ROS_ERROR_COND(ranges.size() <= min || min < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
			pmin.setR(ranges[min]);
			pmin.setPhi(ranges[min]*sin(angleMin+(double)min*angleInc));
			ROS_ERROR_COND(ranges.size() <= min+it->size()-1 || min+it->size()-1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
			pmax.setR(ranges[min+it->size()-1]);
			pmax.setPhi(ranges[min+it->size()-1]*sin(angleMin+angleInc*(double)(min+it->size()-1)));
			d = objectLength(i,min+it->size()-1,tabPoints,ranges,angleMin,angleInc);
		}
		//ROS_INFO("taille objet: %f",d);
		if(d>0.65 && d<0.75)
		{
			Segment segm(pmin,pmax,min,it->size() + min -1);
			ROS_INFO("distance objet %d du laser = %f",i,segm.distanceLaserSegment(ranges));
			ROS_INFO("distance ortho de l objet %d: %f",i,distanceOrtho(segm,ranges,angleMin,angleInc));
			segm.linearRegression(*it);
			ROS_INFO("taille du segment = %f",d); 
			segm.setDistance(d);
			tabSegments.push_back(segm);
		}
		min = min + it->size();
		i++;
	}
	Segment s;
	ROS_INFO("nombre de segment d environ 70 cm : %d",(int)tabSegments.size());
	//ROS_INFO("nearestSegment: %d",nearestSegment(tabSegments,ranges));
	if(tabSegments.size()>1)
	{
		s=tabSegments[nearestSegment(tabSegments,ranges)];
		tabSegments.clear();
		tabSegments.push_back(s);
	}
	return tabSegments;
}

float finalApproaching::objectLength(int i, int j,std::list<std::vector<Point> > tabPoints,std::vector<float> ranges, float angleMin, double angleInc)
{
	/*float length=0;
	if(i!=j)
	{
		ROS_ERROR_COND(ranges.size() <= i || i < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		Point pmin(ranges[i],angleMin+(double)i*angleInc);
		ROS_ERROR_COND(ranges.size() <= j || j < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		Point pmax(ranges[j],angleMin+(double)j*angleInc);
		length = distance2points(pmin,pmax);
	}
	return length;*/
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
		//return distance2points(pmin,pmax);
		return distance2points(tab[0],tab[tab.size()-1]);
	} 
	//return distance2points(m_ranges[cpt2],m_ranges[cpt2+it->size() -1]);
	else
	{
		return 0;
	}
}

int finalApproaching::nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges)
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
float finalApproaching::distanceOrtho(Segment s,std::vector<float> ranges,float angleMin, double angleInc)
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
	ROS_INFO("xg=%f xd=%f",gauche.getX(),droite.getX());
	ROS_INFO("yg=%f yd=%f",gauche.getY(),droite.getY());
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

float finalApproaching::positionYLaser(Segment s,std::vector<float> ranges, float angleMin, double angleInc)
{
	int tmp = 0;
	int min = s.getMinRanges();
	ROS_ERROR_COND(ranges.size() <= min || min < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
	int i = 0;
	int max=s.getMaxRanges();
	ROS_ERROR_COND(ranges.size() <= max || max < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
	for(int i=min; i<max; i++)
	{
		if(ranges[tmp] > ranges[i])
		{
			tmp = i;
		}
	}
	float t = s.getDistance();
	float d = distanceOrtho(s,ranges,angleMin,angleInc);
	while(std::abs(ranges[tmp-i]-ranges[tmp-1-i])<0.03)
	{
		ROS_ERROR_COND(ranges.size() <= tmp-i || tmp-i < 0 || ranges.size() <= tmp-i-1 || tmp-i-1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		i++;
	}
	//ROS_INFO("tmp-i+1: %d m_ranges[tmp-i+1]: %f",tmp-i+1,ranges[tmp-i+1]);
	float left = sqrt(ranges[tmp-i+1]*ranges[tmp-i+1] - d*d);	
	i=0;
	while(std::abs(ranges[tmp+i]-ranges[tmp+i+1])<0.03)
	{
		ROS_ERROR_COND(ranges.size() <= tmp+i || tmp+i < 0 || ranges.size() <= tmp+i+1 || tmp+i+1 < 0, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__, __FILE__);
		i++;
	}
	//ROS_INFO("tmp+i-1: %d m_ranges[tmp+i-1]: %f",tmp+i-1,m_ranges[tmp+i-1]);
	float right = sqrt(ranges[tmp+i-1]*ranges[tmp+i-1] -d*d);
	ROS_INFO("taille: %f ortho: %f",t,d);
	ROS_INFO("left: %f t-right: %f",left,t-right);
	return (left+t-right)/(float)2;
}

std::vector<int> finalApproaching::idWanted(int team,int phase)
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

int finalApproaching::correspondingId(std::vector<int> allPossibleId,std::vector<int> arTagId,std::vector<float> arTagDistance)
{
	int correspondingId = -1;
	std::vector<int> ids;
	ids.clear();
	if(!arTagId.empty())
	{
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
			ROS_INFO("taille de ids: %d de arTagDistance: %d",(int)ids.size(),(int)arTagDistance.size());
			for(int j=0; j<ids.size(); j++)
			{
				ROS_INFO("ids[j]: %d tmp: %d",ids[j],tmp);
				if(arTagDistance[ids[j]]<arTagDistance[tmp])
				{
					tmp = ids[j];
				}
			}
			correspondingId = tmp;
		}
	}
	//ROS_INFO("correspondingId: %d",correspondingId);
	return correspondingId;
}

int finalApproaching::asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz,int k)
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
