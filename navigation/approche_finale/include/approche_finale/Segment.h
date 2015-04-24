#ifndef SEGMENT_H
#define SEGMENT_H

#include "Point.h"
#include <vector>

class Segment{

public:
  Segment(Point a,Point b, int min_r,int max_r);
  ~Segment();
  
  Point get_min(){return m_min;}
  Point get_max(){return m_max;}
  float get_pente(){return m_pente;}
  float get_correlation(){return m_correlation;}
  float get_angle() const{return m_angle;}
  float get_distance(){return m_distance;}
  int get_min_ranges(){return m_min_ranges;}
  int get_max_ranges(){return m_max_ranges;}
  
  void set_pente(float pente){m_pente=pente;}
  void set_correlation(float corr){m_correlation = corr;}
  void set_distance(float d){m_distance = d;}
  
  void regression_lineaire(std::vector<Point>);
  bool pente_nulle();
  
private:
  Point m_min;
  Point m_max;
  float m_pente;
  float m_correlation;
  float m_distance;
  int m_min_ranges;
  int m_max_ranges;
  float m_angle;

};

#endif
