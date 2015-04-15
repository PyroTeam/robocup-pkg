#ifndef SEGMENT_H
#define SEGMENT_H

#include "Point.h"
#include <vector>

class Segment{

public:
  Segment(Point a,Point b);
  ~Segment();
  
  Point get_min(){return m_min;}
  Point get_max(){return m_max;}
  float get_pente(){return m_pente;}
  float get_correlation(){return m_correlation;}
  void set_pente(float pente){m_pente=pente;}
  void set_correlation(float corr){m_correlation = corr;}
  
  void regression_lineaire(std::vector<Point>);
  bool pente_nulle();
  
private:
  Point m_min;
  Point m_max;
  float m_pente;
  float m_correlation;

};

#endif
