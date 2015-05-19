/**
 * \file 			laserScan.h
 * \class			laserScan
 * \brief			classe traitnt les infos du laser 
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef laserScan_H
#define laserScan_H

#include <ros/ros.h>
#include <vector>
#include <list>
#include "sensor_msgs/LaserScan.h"
#include "Point.h"
#include "Segment.h"


class laserScan
{
	public:

		laserScan();
		~laserScan();

/**
 *  \brief		Cree les objets a partir du tableau de points
 */
		void Objects();
		
		void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
		
/**
 *  \brief		determine l objet contenant le maximum de points
 *  \return		le numero de l'objet qui contient le plus de points
 */
		int maxNumberPoints();
		
/**
 *  \brief		Calcule la longueur d un objet
 *  \return		la longueur de l objet
 */
		float length(int i,int j);
		
/**
 *  \brief		determine la distance entre l' objet et le laser
 *  \return		la distanc entre l objet et le laser
 */
		float distanceObject(Segment s);
		
/**
 *  \brief		Determine le segment le plus proche du laser
 *  \return		le numero du  segment le plus proche du laser
 */
		int nearestSegment();
		
/**
 *  \brief		construit les différents segments
 */
		void buildSegments();
		
/**
 *  \brief		position en y (repere laser) du bord droit du segment
 *  \return		distance entre le bord droit de la machine et le projete orthogonal du laser sur la machine
 */
		float positionY(Segment s);
		
/**
 *  \brief		Determine la distance orthogonale entre le laser et le segment
 *  \return		la distance orthogonale entre le laser et le segment
 */
		float distanceOrtho(Segment s);

		float getRangeMin(){return m_rangeMin;}
		float getRangeMax(){return m_rangeMax;}
		float getAngleMin(){return m_angleMin;}
		float getAngleMax(){return m_angleMax;}
		double getAngleInc(){return m_angleInc;}
		std::vector<float>& getRanges() {return m_ranges;}
		std::list<std::vector<Point> > getTabPoints() {return m_tabPoints;}
		std::vector<Segment> getTabSegments() {return m_tabSegments;}

		void setRangeMin(float min){m_rangeMin=min;}
		void setRangeMax(float max){m_rangeMax=max;}
		void setAngleMin(float min){m_angleMin=min;}
		void setAngleMax(float max){m_angleMax=max;}
		void setAngleInc(double inc){m_angleInc=inc;}


	private:

		std::vector<float> m_ranges;
		std::list<std::vector<Point> > m_tabPoints;
		std::vector<Segment> m_tabSegments;
		float m_rangeMin;
		float m_rangeMax;
		float m_angleMin;
		float m_angleMax;
		double m_angleInc;
		ros::NodeHandle m_nh;
		ros::Subscriber m_lsSub;

};
#endif
