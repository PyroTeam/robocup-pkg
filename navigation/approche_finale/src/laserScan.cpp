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

void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
	m_tabpoints.clear();
	m_tabsegments.clear();
  setRangeMin(scan->range_min);
  setRangeMax(scan->range_max);
  setAngleMin(scan->angle_min);
  setAngleMax(scan->angle_max);
  setAngleInc(scan->angle_increment);
  m_ranges = scan->ranges;
	Objects();
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
	std::cout << "\n\nnombre d elements de tabpoints "<<m_tabpoints.size()<< std::endl;
	int j=0,cpt=0;
	for(it=m_tabpoints.begin();it!=m_tabpoints.end();it++){
		std::cout << "taille de l objet "<<j<<" = "<< it->size() << std::endl;
		//float d = length(j);
		//std::vector<Point> tab = *it;
		//if(d>0.65 && d<0.75){
		//	Segment s(tab[0],tab[tab.size()-1],cpt,cpt+it->size()-1);
		//	s.regression_lineaire(*it);
		//	m_tabsegments.push_back(s);
		//}
		j++;
		//cpt = cpt + it->size();
	}
	build_segments();
	std::cout << "nombre de segment d environ 70 cm : " << m_tabsegments.size() <<"\n"<< std::endl;
}

void laserScan::build_segments(){
  std::list<std::vector<Point> >::iterator it;
  int min=0,i=0;
  for(it = m_tabpoints.begin();it != m_tabpoints.end(); it++){
  	std::vector<Point> tab = *it;
	Point pmin(m_ranges[min],m_ranges[min]*sin(m_angle_min+(double)min*m_angle_inc));
	Point pmax(m_ranges[min+it->size()-1],m_ranges[min+it->size()-1]*sin(m_angle_min+m_angle_inc*(double)(min+it->size()-1)));
	Segment s(pmin,pmax,min,it->size() + min -1);
  	std::cout << "min_build: "<<min<<" max_build: "<<min +it->size() -1<<std::endl;
	std::cout << "m_ranges[min_build]: "<<m_ranges[min]<<" m_ranges[max_build]: "<<m_ranges[min+it->size()-1]<<std::endl;
	float d = length(i,min+it->size()); 
  	std::cout << "taille objet: " << d << std::endl;
	std::cout << "distance objet "<< i <<" du laser = " << distance_objet(s) << std::endl;
  	if(d>0.65 && d<0.75){
			s.regression_lineaire(*it);
			std::cout << "taille du segment = "<< d << std::endl; 
			s.set_distance(d);
			m_tabsegments.push_back(s);
		}
		min = min + it->size();
		i++;
	}
}


float laserScan::length(int i,int j){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
	int cpt2 =0;
  while(compteur!=i){

    compteur++;
	cpt2 = cpt2 + it->size(); 
	it++;
  }
	std::cout << "min: " << cpt2 << " max: " << cpt2+it->size() -1 << std::endl;
  std::vector<Point> tab = *it;
Point pmin(m_ranges[i],m_ranges[i]+(float)i*sin((double)i*m_angle_inc));
Point pmax(m_ranges[j],m_ranges[j]+(float)j*sin((double)j*m_angle_inc));
//return distance2points(pmin,pmax);
  return distance2points(tab[0],tab[tab.size()-1]); 
//	return distance2points(m_ranges[cpt2],m_ranges[cpt2+it->size() -1]);
}


float laserScan::distance_objet(Segment s){
	int min = s.get_min_ranges();
	int max = s.get_max_ranges();
	return m_ranges[(max+min)/2];  
}

int laserScan::nearest_segment(){
	int tmp = 0;
	std::vector<Segment>::iterator it;
	for(int i=0; i<m_tabsegments.size(); i++){
		if(distance_objet(m_tabsegments[i]) < distance_objet(m_tabsegments[tmp]))
			tmp = i;
	}
	return tmp;
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


float laserScan::distance_ortho(Segment s){
	int min = s.get_min_ranges();
	int max = s.get_max_ranges();
	float tmp = s.get_min_ranges();
	for(int i=min;i<=max;i++){
		if(m_ranges[tmp] > m_ranges[i]){
			tmp = i;
		}
	}
	if(std::abs(tmp-min) < 5){
		std::cout << "erreur aller vers la droite " <<std::endl;
		return -1;
	}
	if(std::abs(tmp-max) < 5){
		std::cout << "erreur aller vers la gauche " <<std::endl;
		return -2;
	}
	return m_ranges[tmp];
}

float laserScan::position_y(Segment s){
	int min = s.get_min_ranges();
	int i = 0;
	if(m_ranges[min]>1.0){
	while(std::abs(m_ranges[min+i]-m_ranges[min+i+1]) < 0.03){
		i++;
	}
	}
	int max = s.get_max_ranges();
	while(std::abs(m_ranges[max-i]-m_ranges[max-i-1] < 0.03)){
		i++;
	}
	min = max -i;
	float t = s.get_distance();
	float d = distance_ortho(s);
	float d_milieu = distance_objet(s);
	float gauche = sqrt(m_ranges[min]*m_ranges[min] - d*d);	
	
	float droite = sqrt(m_ranges[max]*m_ranges[max] -d*d);
	std::cout << "min: " << min << " max: " << max << " distance milieu: " << d_milieu << std::endl;
	std::cout << "taille: " << t << " ortho: " << d << std::endl;
	std::cout << "gauche: " << gauche << " t-droite: " << t-droite << std::endl;
	return t-droite;
	//return (gauche+t-droite)/(float)2;
}




