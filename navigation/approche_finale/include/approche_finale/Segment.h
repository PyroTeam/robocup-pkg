/**
 * \file 			Segment.h
 * \class			Segment
 * \brief			classe représentant un segment d environ 70 cm
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef SEGMENT_H
#define SEGMENT_H

#include "Point.h"
#include <vector>

class Segment{

	public:
		Segment();  
		Segment(Point a,Point b, int minR,int maxR);
		~Segment();
 
		Point getMin(){return m_min;}
		Point getMax(){return m_max;}
		float getGradient(){return m_gradient;}
		float getCorrelation(){return m_correlation;}
		float getAngle(){return m_angle;}
		float getDistance(){return m_distance;}
		int getMinRanges(){return m_minRanges;}
		int getMaxRanges(){return m_maxRanges;}
		
		void setGradient(float gradient){m_gradient=gradient;}
		void setCorrelation(float corr){m_correlation = corr;}
		void setDistance(float d){m_distance = d;}

/**
 *  \brief		effectue un regression lineaire sur le segment
 */	 
		void linearRegression(std::vector<Point>);
		
/**
 *  \brief		determine si la pente est quasi nulle ou non
 *  \return		true si pente nulle sinon false
 */
		bool nilGradient();
		
		float distanceLaserSegment(std::vector<float> ranges);
	  
	private:
		Point m_min;
		Point m_max;
		float m_gradient;
		float m_correlation;
		float m_distance;
		int m_minRanges;
		int m_maxRanges;
		float m_angle;

};

#endif
