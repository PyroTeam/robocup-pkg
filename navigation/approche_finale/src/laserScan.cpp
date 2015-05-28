#include "laserScan.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "Point.h"
#include "fa_utils.h"
#include "Segment.h"
#include "sensor_msgs/LaserScan.h"

laserScan::laserScan():m_rangeMin(0.0),m_rangeMax(0.0),m_angleMin(0.0),m_angleMax(0.0),m_angleInc(0.0)
{
	m_lsSub = m_nh.subscribe("/scan",1000,&laserScan::laserCallback,this);
}

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	m_tabPoints.clear();
	m_tabSegments.clear();
	setRangeMin(scan->range_min);
	setRangeMax(scan->range_max);
	setAngleMin(scan->angle_min);
	setAngleMax(scan->angle_max);
	setAngleInc(scan->angle_increment);
	m_ranges = scan->ranges;
	Objects();
}


laserScan::~laserScan(){}

void laserScan::Objects()
{
	Point p0(m_ranges[0],m_angleMin);
	std::vector<Point> tabP0;
	tabP0.push_back(p0);
	m_tabPoints.push_back(tabP0);
	std::list<std::vector<Point> >::iterator it = m_tabPoints.begin();
	for (int i=1; i<m_ranges.size(); i++)
	{
		if((m_ranges[i]>m_rangeMin) && (m_ranges[i]<m_rangeMax))
		{
			Point p(m_ranges[i],m_angleMin + (float)i*m_angleInc);
			if(std::abs(m_ranges[i] - m_ranges[i-1]) < 0.05)
			{
				it->push_back(p);
			}
			else
			{
				std::vector<Point> tabP;
				tabP.push_back(p);
				m_tabPoints.push_back(tabP);
				it++;
			}
		}
	}
	/*for(it=m_tabPoints.begin();it!=m_tabPoints.end();it++)
	{
		if(it->size()<10)
		{
			m_tabPoints.erase(it);
			it--;
		}
	}*/
	ROS_INFO("nombre d elements de tabPoints: %d",(int)m_tabPoints.size());
	int j=0,cpt=0;
	for(it=m_tabPoints.begin();it!=m_tabPoints.end();it++)
	{
		ROS_INFO("nombre de points de l objet: %d = %d",j,(int)it->size());
		//float d = length(j);
		//std::vector<Point> tab = *it;
		//if(d>0.65 && d<0.75)
		//{
		//	Segment s(tab[0],tab[tab.size()-1],cpt,cpt+it->size()-1);
		//	s.linearRegression(*it);
		//	m_tabSegments.push_back(s);
		//}
		j++;
		//cpt = cpt + it->size();
	}
	//buildSegments();
	//ROS_INFO("nombre de segment d environ 70 cm : %d",(int)m_tabSegments.size());
}

void laserScan::buildSegments()
{
	std::list<std::vector<Point> >::iterator it;
	int min=0,i=0;
	for(it = m_tabPoints.begin();it != m_tabPoints.end(); it++)
	{
		float d=0;
		Point pmin(0,0);
		Point pmax(0,0);
		if(it->size()>1)
		{
			pmin.setR(m_ranges[min]);
			pmin.setPhi(m_ranges[min]*sin(m_angleMin+(double)min*m_angleInc));
			pmax.setR(m_ranges[min+it->size()-1]);
			pmax.setPhi(m_ranges[min+it->size()-1]*sin(m_angleMin+m_angleInc*(double)(min+it->size()-1)));
		d = length(i,min+it->size()-1);
		}
		ROS_INFO("taille objet: %f",d);
		if(d>0.65 && d<0.75)
		{
			Segment segm(pmin,pmax,min,it->size() + min -1);
			ROS_INFO("distance objet %d du laser = %f",i,distanceObject(segm));
			segm.linearRegression(*it);
			ROS_INFO("taille du segment = %f",d); 
			segm.setDistance(d);
			m_tabSegments.push_back(segm);
		}
		min = min + it->size();
		i++;
	}
	//Segment s;
	if(m_tabSegments.size()>0)
	{
		m_mainSegment = m_tabSegments[nearestSegment()];
		/*s = m_tabSegments[nearestSegment()];
		m_tabSegments.clear();
		m_tabSegments.push_back(s);*/
		
	}
}


float laserScan::length(int i,int j)
{
	std::list<std::vector<Point> >::iterator it = m_tabPoints.begin();
	int compteur=0;
	int cpt2 =0;
	while(compteur!=i)
	{
		compteur++;
		cpt2 = cpt2 + it->size(); 
		it++;
	}
	if(i!=j)
	{
		std::vector<Point> tab = *it;
		Point pmin(m_ranges[i],m_angleMin+(double)i*m_angleInc);
		Point pmax(m_ranges[j],m_angleMin+(double)j*m_angleInc);
		//return distance2points(pmin,pmax);
		return distance2points(tab[0],tab[tab.size()-1]);
	} 
	//return distance2points(m_ranges[cpt2],m_ranges[cpt2+it->size() -1]);
	else
	{
		return 0;
	}
}


float laserScan::distanceObject(Segment s)
{
	int min = s.getMinRanges();
	int max = s.getMaxRanges();
	return m_ranges[(max+min)/2];  
}

int laserScan::nearestSegment()
{
	int tmp = 0;
	std::vector<Segment>::iterator it;
	for(int i=0; i<m_tabSegments.size(); i++)
	{
		if(distanceObject(m_tabSegments[i]) < distanceObject(m_tabSegments[tmp]))
		{
			tmp = i;
		}
	}
	/*for(int j=0;j<m_tabSegments.size(); j++)
	{
		if(j != tmp)
		{
			m_tabSegments[j] = m_tabSegments[tmp];
		
		}
	}*/
	return tmp;
}



int laserScan::maxNumberPoints()
{
	int tmp = m_tabSegments[0].getMaxRanges()-m_tabSegments[0].getMinRanges();
	for(int i=0;i<m_tabSegments.size();i++)
	{
		if((m_tabSegments[i].getMaxRanges()-m_tabSegments[i].getMinRanges())>tmp)
		{
			tmp = i;
		}
	}
	return tmp;
}

//condition préalable: la machine est a 90° du laser
float laserScan::distanceOrtho(Segment s)
{
	float ortho=0.0;
	int min = s.getMinRanges();
	int max = s.getMaxRanges();
	Point gauche(m_ranges[max],m_angleMin+(double)max*m_angleInc);
	Point droite(m_ranges[min],m_angleMin+(double)min*m_angleInc);
	//si le laser se trouve entre les deux points extremes du segment
	//si yg et yd sont de signes différents
	ROS_INFO("xg=%f xd=%f",gauche.getX(),droite.getX());
    ROS_INFO("yg=%f yd=%f",gauche.getY(),droite.getY());
	if(gauche.getY()*droite.getY()<0)
	{
		int tmp = min;
		for(int i=min;i<=max;i++)
		{
			if(m_ranges[tmp] > m_ranges[i])
			{
				tmp = i;
			}
		}
		ortho = m_ranges[tmp];
		ROS_INFO("ortho: %f",ortho); 
	}
	else
	{
		float orthoMin = m_ranges[min]*cos(m_angleMin+(double)min*m_angleInc);
		float orthoMax = -m_ranges[max]*cos(m_angleMax+(double)max*m_angleInc);
		ortho = (orthoMin+orthoMax)/(float)2;
		ROS_INFO("orthoMin: %f orthoMax: %f ortho: %f",orthoMin,orthoMax,ortho);
	}
	return ortho;
	
	/*
	if(std::abs(tmp-min) < 5)
	{
		ROS_INFO("erreur aller vers la right ");
		return -1;
	}
	if(std::abs(tmp-max) < 5)
	{
		ROS_INFO("erreur aller vers la left ");
		return -2;
	}
	return m_ranges[tmp];*/
}

float laserScan::positionY(Segment s)
{
	int tmp = 0;
	int min = s.getMinRanges();
	int i = 0;
	int max=s.getMaxRanges();
	for(int i=min; i<max; i++)
	{
		if(m_ranges[tmp] > m_ranges[i])
		{
			tmp = i;
		}
	}
	int med=(min+max)/2;
	/*if(m_ranges[min]>1.0)
	{
		while(std::abs(m_ranges[min+i]-m_ranges[min+i+1]) < 0.03)
		{
			i++;
		}
	}
	int max = s.getMaxRanges();
	while(std::abs(m_ranges[max-i]-m_ranges[max-i-1] < 0.03))
	{
		i++;
	}
	min = max -i;*/
	float t = s.getDistance();
	float d = distanceOrtho(s);
	float dMiddle = distanceObject(s);
	while(std::abs(m_ranges[tmp-i]-m_ranges[tmp-1-i])<0.03)
	{
		i++;
	}
	ROS_INFO("tmp-i+1: %d m_ranges[tmp-i+1]: %f",tmp-i+1,m_ranges[tmp-i+1]);
	float left = sqrt(m_ranges[tmp-i+1]*m_ranges[tmp-i+1] - d*d);	
	//return left;
	i=0;
	while(std::abs(m_ranges[tmp+i]-m_ranges[tmp+i+1])<0.03)
	{
		i++;
	}
	ROS_INFO("tmp+i-1: %d m_ranges[tmp+i-1]: %f",tmp+i-1,m_ranges[tmp+i-1]);
	float right = sqrt(m_ranges[tmp+i-1]*m_ranges[tmp+i-1] -d*d);
	ROS_INFO("taille: %f ortho: %f",t,d);
	ROS_INFO("left: %f t-right: %f",left,t-right);
	//return t-right;
	return (left+t-right)/(float)2;
}
