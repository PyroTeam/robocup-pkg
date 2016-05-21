/**
 * \file
 * \class      Segment
 * \brief      Classe représentant un segment d'environ 70 cm
 * \author     Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date       2015-05-25
 * \copyright  PyroTeam, Polytech-Lille
 */

#ifndef _FINAL_APPROACH__SEGMENT__H_
#define _FINAL_APPROACH__SEGMENT__H_

#include "Point.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include <vector>

class Segment{

	public:
		Segment();
		Segment(geometry_msgs::Point min, geometry_msgs::Point max, float gradient);
		Segment(Point a, Point b, int minR, int maxR);
		~Segment();

		geometry_msgs::Point getMinPoint(){return m_minPoint;}
		geometry_msgs::Point getMaxPoint(){return m_maxPoint;}
		Point getMin(){return m_min;}
		Point getMax(){return m_max;}
		float getGradient(){return m_gradient;}
		float getCorrelation(){return m_correlation;}
		float getAngle(){return m_angle;}
		float getDistance(){return m_distance;}
		int getMinRanges(){return m_minRanges;}
		int getMaxRanges(){return m_maxRanges;}

		void setMinPoint(geometry_msgs::Point p){m_minPoint = p;}
		void setMaxPoint(geometry_msgs::Point p){m_maxPoint = p;}
		void setGradient(float gradient){m_gradient = gradient;}
		void setCorrelation(float corr){m_correlation = corr;}
		void setDistance(float d){m_distance = d;}

		/**
		 * \brief      Effectue un regression lineaire sur le segment
		 *
		 * \param[in]  <unnamed>  { parameter_description }
		 *
		 * \return     { description_of_the_return_value }
		 */
		geometry_msgs::Pose2D linearRegression(std::vector<Point>);

		/**
		 * \brief      Determine si la pente est quasi nulle ou non
		 *
		 * \return     true si pente nulle sinon false
		 */
		bool nilGradient();

		/**
		 * \brief      Determine la distance entre le segment et le laser
		 *
		 * \param[in]  ranges  The ranges
		 *
		 * \return     la distance entre l'objet et le laser
		 */
		float distanceLaserSegment(std::vector<float> ranges);

	private:
		geometry_msgs::Point m_minPoint;
		geometry_msgs::Point m_maxPoint;
		Point m_min;
		Point m_max;
		float m_gradient;
		float m_correlation;
		float m_distance;
		int m_minRanges;
		int m_maxRanges;
		float m_angle;
};

#endif // _FINAL_APPROACH__SEGMENT__H_
