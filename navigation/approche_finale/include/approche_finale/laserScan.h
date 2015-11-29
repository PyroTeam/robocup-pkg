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
#include "sensor_msgs/LaserScan.h"


class laserScan
{
	public:

		laserScan();
		~laserScan();
		
		void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
		
		float getRangeMin(){return m_rangeMin;}
		float getRangeMax(){return m_rangeMax;}
		float getAngleMin(){return m_angleMin;}
		float getAngleMax(){return m_angleMax;}
		double getAngleInc(){return m_angleInc;}
		std::vector<float>& getRanges() {return m_ranges;}
		
		void setRangeMin(float min){m_rangeMin=min;}
		void setRangeMax(float max){m_rangeMax=max;}
		void setAngleMin(float min){m_angleMin=min;}
		void setAngleMax(float max){m_angleMax=max;}
		void setAngleInc(double inc){m_angleInc=inc;}


	private:

		std::vector<float> m_ranges;
		float m_rangeMin;
		float m_rangeMax;
		float m_angleMin;
		float m_angleMax;
		double m_angleInc;
		ros::NodeHandle m_nh;
		ros::Subscriber m_lsSub;

};
#endif
