#include "finalApproaching.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <manager_msg/finalApproachingAction.h>
#include <cmath>
#include <vector>
#include <list>
#include "bumperlistener.h"
#include "geometry_msgs/Twist.h"
#include "fa_utils.h"
#include "laserScan.h"
#include "Point.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int a=0,b=0,c=0,d=0,cpt=0;
float a1=0.0;

finalApproaching::~finalApproaching(void){}

void finalApproaching::executeCB(const manager_msg::finalApproachingGoalConstPtr &goal)
{
	// helper variables
	ros::Rate loopRate(100);
	m_pubMvt = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	//m_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker",1000);
	bool success = true;
	BumperListener bp;

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
	   		  
	laserScan ls;
	loopRate.sleep();
	float positionY=0;
	float gradient=0;
	float ortho=0;
	float moyY=0;
	float moyO=0;
	int maxPoints=0;
	int nearby=0;
	int j = 0;
	std::list<float> listPositionY;
	std::list<float> listOrtho;
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
	ROS_INFO("Waiting a complete laserscan");
	while(ros::ok() && !bp.getState() && c!=2)
	{
		if(ls.getRanges().size() == 513)
		{
			std::vector<float> ranges = ls.getRanges();
			float angleMin = ls.getAngleMin();
			double angleInc = ls.getAngleInc();
			float rangeMin = ls.getRangeMin();
			float rangeMax = ls.getRangeMax();
			std::list<std::vector<Point> > tabPoints = ls.getTabPoints();
			std::vector<Segment> tabSegments = segmentsConstruction(tabPoints, ranges, angleMin, angleInc);
			if(tabSegments.size()>0)
			{
				Segment seg = tabSegments[nearestSegment(tabSegments,ranges)];
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
					if(j<20)
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
						if(a == 1 && d != 2)
						{
							d = asservissementPositionX(m_pubMvt,ortho,0.50);
						}
						if((a == 1) && (d == 2) && (cpt < 200))
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
	if(bp.getState())
	{
		success=false;
		geometry_msgs::Twist stop;
		a=0;
		b=0;
		c=0;
		d=0;
		cpt=0;
		j=0;
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
		d=0;
		cpt=0;
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
			tmp = 0.4; break;
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
		tmp = 0.145;
	}
	return tmp;
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
			pmin.setR(ranges[min]);
			pmin.setPhi(ranges[min]*sin(angleMin+(double)min*angleInc));
			pmax.setR(ranges[min+it->size()-1]);
			pmax.setPhi(ranges[min+it->size()-1]*sin(angleMin+angleInc*(double)(min+it->size()-1)));
		d = objectLength(min,min+it->size()-1,ranges,angleMin,angleInc);
		}
		ROS_INFO("taille objet: %f",d);
		if(d>0.65 && d<0.75)
		{
			Segment segm(pmin,pmax,min,it->size() + min -1);
			ROS_INFO("distance objet %d du laser = %f",i,segm.distanceLaserSegment(ranges));
			segm.linearRegression(*it);
			ROS_INFO("taille du segment = %f",d); 
			segm.setDistance(d);
			tabSegments.push_back(segm);
		}
		min = min + it->size();
		i++;
	}
	//Segment s;
	ROS_INFO("nombre de segment d environ 70 cm : %d",(int)tabSegments.size());
	return tabSegments;
}

float finalApproaching::objectLength(int i, int j,std::vector<float> ranges, float angleMin, double angleInc)
{
	float length=0;
	if(i!=j)
	{
		Point pmin(ranges[i],angleMin+(double)i*angleInc);
		Point pmax(ranges[j],angleMin+(double)j*angleInc);
		length = distance2points(pmin,pmax);
	}
	return length;
}

int finalApproaching::nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges)
{
	int nearest=0;
	std::vector<Segment>::iterator it;
	for(int i=0; i<tabSegments.size(); i++)
	{
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
	Point gauche(ranges[max],angleMin+(double)max*angleInc);
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
	while(std::abs(ranges[tmp-i]-ranges[tmp-1-i])<0.03)
	{
		i++;
	}
	//ROS_INFO("tmp-i+1: %d m_ranges[tmp-i+1]: %f",tmp-i+1,ranges[tmp-i+1]);
	float left = sqrt(ranges[tmp-i+1]*ranges[tmp-i+1] - d*d);	
	i=0;
	while(std::abs(ranges[tmp+i]-ranges[tmp+i+1])<0.03)
	{
		i++;
	}
	//ROS_INFO("tmp+i-1: %d m_ranges[tmp+i-1]: %f",tmp+i-1,m_ranges[tmp+i-1]);
	float right = sqrt(ranges[tmp+i-1]*ranges[tmp+i-1] -d*d);
	ROS_INFO("taille: %f ortho: %f",t,d);
	ROS_INFO("left: %f t-right: %f",left,t-right);
	return (left+t-right)/(float)2;
}
