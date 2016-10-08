/**
 * \file
 * \class      Segment
 * \brief      Classe représentant un segment d'environ 70 cm
 * \author     Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date       2015-05-25
 * \copyright  2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _FINAL_APPROACH__SEGMENT__H_
#define _FINAL_APPROACH__SEGMENT__H_

#include "Point.h"
#include "geometry_utils/geometry_utils.h"

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
		geometry_msgs::Point getMiddlePoint(){return m_middlePoint;}
		Point getMin(){return m_min;}
		Point getMax(){return m_max;}
		double getCorrelation(){return m_correlation;}
		double getAngle(){return m_angle;}
		double getDistance(){return m_distance;}
		int getMinRanges(){return m_minRanges;}
		int getMaxRanges(){return m_maxRanges;}
		bool isConstructed(){return !m_nullSegment;}

		void setMinPoint(geometry_msgs::Point p){m_minPoint = p;}
		void setMaxPoint(geometry_msgs::Point p){m_maxPoint = p;}
		void setCorrelation(float corr){m_correlation = corr;}
		void setDistance(float d){m_distance = d;}

		/**
		 * \brief      Effectue un regression lineaire sur le segment
		 *
		 * \param[in]  pointsVector  The points vector
		 *
		 * \return     { description_of_the_return_value }
		 */
		geometry_msgs::Pose2D linearRegression(std::vector<Point> pointsVector);


		/**
		 * \brief      Determine la distance entre le segment et le laser
		 *
		 * \return     la distance entre l'objet et le laser
		 */
		float distanceLaserSegment();

		/**
		 * \brief      Determine la distance orthogonale (sur axe x laser) entre le segment et le laser
		 *
		 * \return     la distance entre l'objet et le laser
		 */
		float distanceOrthoLaserSegment();

	private:
		geometry_msgs::Point m_minPoint;
		geometry_msgs::Point m_maxPoint;

		geometry_msgs::Point m_middlePoint;  // CONFIANCE
		geometry_msgs::Point m_orthoPoint;  // CONFIANCE

		Point m_min;
		Point m_max;
		double m_correlation;
		double m_distance;
		int m_minRanges;
		int m_maxRanges;
		double m_angle;		 // CONFIANCE
		bool m_nullSegment;  // CONFIANCE
};

#endif // _FINAL_APPROACH__SEGMENT__H_
