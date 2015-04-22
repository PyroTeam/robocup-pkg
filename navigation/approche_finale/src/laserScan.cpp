// d apres le code de Thomas Danel

#include "laserScan.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "Point.h"
#include "fa_utils.h"
#include "Segment.h"
#include "sensor_msgs/LaserScan.h"

laserScan::laserScan():m_range_min(0.0),m_range_max(0.0),m_angle_min(0.0),m_angle_max(0.0),m_angle_inc(0.0){
	m_ls_sub = m_nh.subscribe("/scan",1000,&laserScan::laserCallback,this);
}

laserScan::~laserScan(){
}

void laserScan::Objects(){
	Point p0(m_ranges[0],m_angle_min);
  std::vector<Point> tabP0;
  tabP0.push_back(p0);
  m_tabpoints.push_back(tabP0);
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  for (int i=1; i<m_ranges.size(); i++){
    if((m_ranges[i]>m_range_min) && (m_ranges[i]<m_range_max)){
      Point p(m_ranges[i],m_angle_min + (float)i*m_angle_inc);
      if(std::abs(m_ranges[i] - m_ranges[i-1]) < 0.05){
        it->push_back(p);
        }
      else{
        std::vector<Point> tabP;
        tabP.push_back(p);
        m_tabpoints.push_back(tabP);
        it++;
      }
    }
  }
  for(it=m_tabpoints.begin();it!=m_tabpoints.end();it++){
  	if(it->size()<10){
  		m_tabpoints.erase(it);
  		it--;
		}
	}
}


void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
  m_tabpoints.clear();
  setRangeMin(scan->range_min);
  setRangeMax(scan->range_max);
  setAngleMin(scan->angle_min);
  setAngleMax(scan->angle_max);
  setAngleInc(scan->angle_increment);
  m_ranges = scan->ranges;
	Objects();
}

float laserScan::length(int i){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  while(compteur!=i){
    it++;
    compteur++;
  }
  std::vector<Point> tab = *it;
  return distance2points(tab[0],tab[tab.size()-1]); 
}

bool laserScan::nearby_object(int i){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  int min=0; //to know how many ranges
  while(compteur!=i){
    min = min + it->size();
    it++;
    compteur++;
  }
  int max = min + it->size();
  int med = (max + min)/2;
  if(m_ranges[med] < 0.30)
    return true;
  else
    return false;   
}

float laserScan::distance_objet(int i){
	std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  int min=0; //to know how many ranges
  while(compteur!=i){
    min = min + it->size();
    it++;
    compteur++;
  }
  float tmp = min;
  for(int i=min;i<(min+it->size()-1);i++){
  	if(m_ranges[tmp]>m_ranges[i]){
  		tmp = i;
  		}
  }
  return m_ranges[tmp];  
}

int laserScan::max_number_points(){
  std::vector<Point> tmp = m_tabpoints.front();
  std::list<std::vector<Point> >::iterator it;
  int compteur=0;
  for(it = m_tabpoints.begin();it != m_tabpoints.end(); it++){
    if(it->size() > tmp.size()){
      tmp = *it;
      compteur++;
    }
  }
  return compteur;
}

int laserScan::nearest_object(){
	std::list<std::vector<Point> >::iterator it;
	int proche = 0;
	int min = 0;
	int max = m_tabpoints.begin()->size() -1;
	float tmp = (m_ranges[min]+m_ranges[max])/(float)2;
	for(it = m_tabpoints.begin(); it != m_tabpoints.end(); it++){ 
		if(m_ranges[min]+m_ranges[max]/(float)2 < tmp){
			proche++;
			tmp = it->begin()->getr(); 	
		}
		min = max + 1;
		max = max + it->size();
	}
	return proche;
}

Segment laserScan::build_segment(int i){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  int min=0; //first ranges of the segment
  while(compteur!=i){
    min = min + it->size();
    it++;
    compteur++;
  }
  int max = min + it->size() - 1;
  Segment s(m_ranges[min],m_ranges[max]);
  s.regression_lineaire(*it);
  return s;
}

float laserScan::position_y(int i,float d){
	std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  int min=0; //first ranges of the segment
  while(compteur!=i){
    min = min + it->size();
    it++;
    compteur++;
  }
  int max = min + it->size() - 1;
	float gauche = sqrt(m_ranges[min]*m_ranges[min] - d*d);
	//float droite = sqrt((0.70-m_ranges[max])*(0.70-m_ranges[max]) - d*d);
	//return (gauche+droite)/(float)2;
	if(gauche < 0.70)
		return gauche;
	else
		return 0.0;
}




