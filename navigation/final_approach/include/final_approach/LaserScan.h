/**
 * \file 			LaserScan.h
 * \class			LaserScan
 * \brief			classe traitnt les infos du laser
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _FINAL_APPROACH__LASERSCAN__H_
#define _FINAL_APPROACH__LASERSCAN__H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>
#include <string>


class LaserScan
{
	public:
		LaserScan();
		~LaserScan();

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
		ros::Time getStamp(void) {return m_stamp;}
		std::string getFrame(void) {return m_frame;}

	private:
		std::vector<float> m_ranges;
		float m_rangeMin;
		float m_rangeMax;
		float m_angleMin;
		float m_angleMax;
		double m_angleInc;
		ros::NodeHandle m_nh;
		ros::Subscriber m_lsSub;
		ros::Time m_stamp;
		std::string m_frame;
};
#endif // _FINAL_APPROACH__LASERSCAN__H_
