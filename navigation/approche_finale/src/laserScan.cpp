// d apres le code de Thomas Danel

#include "laserScan.h"
#include <cmath>
#include "Point.h"
#include "fa_utils.h"

laserScan::laserScan():m_range_min(0.0),m_range_max(0.0),m_angle_min(0.0),m_angle_max(0.0),m_angle_inc(0.0){
  Point p0(m_ranges[0],getAngleMin());
  std::vector<Point> tabP0;
  tabP0.push_back(p0);
  m_tabpoints.push_back(tabP0);
}

laserScan::~laserScan(){
}

void laserScan::Objects(){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.end();
  for (int i=1; i<m_ranges.size(); i++){
    if((m_ranges[i]>getRangeMin()) && (m_ranges[i]<getRangeMax())){
      Point p(m_ranges[i],getAngleMin() + (float)i*getAngleInc());
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
}


void laserScan::laserCallback(const sensor_msgs::LaserScanConstPtr& scan){
  m_tabpoints.clear();
  setRangeMin(scan->range_min);
  setRangeMax(scan->range_max);
  setAngleMin(scan->angle_min);
  setAngleMax(scan->angle_max);
  setAngleInc(scan->angle_increment);
  m_ranges=scan->ranges;
  Objects();
}

int laserScan::max_number_points(){
  std::vector<Point> tmp = m_tabpoints.front();
  std::list<std::vector<Point> >::iterator it;
  for(it = m_tabpoints.begin();it != m_tabpoints.end(); it++){
    if(it->size() > tmp.size()){
      tmp = *it;
    }
  }
  return tmp.size();
}

bool laserScan::around_70cm(int i){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  while(compteur!=i){
    it++;
    compteur++;
  }
  float d = distance2points(*it->begin(),*it->end()); 
  if(d > 0.65 && d < 0.75){
    return true;
  }
  else{
    return false;
  }
}

bool laserScan::nearby_object(int i){
  std::list<std::vector<Point> >::iterator it = m_tabpoints.begin();
  int compteur=0;
  int compteur2=0; //to know how many ranges
  while(compteur!=i){
    it++;
    compteur++;
    compteur2 = compteur2 + it->size();
  }
  int max = it->size();
  if((max%2)==0)
    max = max--;
  int med = max/2 + 1;
  med = med + compteur2;
  if(m_ranges[med] < 0.30)
    return true;
  else
    return false;   
}
