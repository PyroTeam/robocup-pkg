/**
 * \file 			finalApproaching.h
 * \class			finalApproaching
 * \brief			classe principale de l approche finale
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef FINALAPPROACHING_H
#define FINALAPPROACHING_H

#include <manager_msg/finalApproachingAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <vector>
#include "Point.h"
#include "Segment.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class finalApproaching
{

	protected:
		ros::NodeHandle nh;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<manager_msg::finalApproachingAction> as;
		std::string actionName;
		// create messages that are used to published feedback/result
		manager_msg::finalApproachingFeedback feedback;
		manager_msg::finalApproachingResult result;
		ros::Publisher m_pubMvt;
		ros::Publisher m_markerPub;
		int m_type;
		int m_side;
		int m_parameter;
		

	public:
		finalApproaching(std::string name) :
		as(nh, name, boost::bind(&finalApproaching::executeCB, this, _1), false),actionName(name)
		{
			as.start();
		}

		~finalApproaching(void);

		void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
		
/**
 *  \brief		avancement de l action
 *  \return		un entier representant l evolution de l action
 */		
		int avancement(int a, int b, int c);
		
/**
 *  \brief		determine la consigne en x (repere laser)
 *  \return		consigne en x
 */
		float objectifX();
		
/**
 *  \brief		determine la consigne en y (repere laser)
 *  \return		consigne en y
 */
		float objectifY();
		
		std::list<std::vector<Point> > objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc, float rangeMin, float rangeMax);
		
		std::vector<Segment> segmentsConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc);
		
		float objectLength(int i, int j, std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc);
		
		int nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges);
		
		float distanceOrtho(Segment s,std::vector<float> ranges,float angleMin, double angleInc);
		
		float positionYLaser(Segment s,std::vector<float> ranges, float angleMin, double angleInc);
		
		std::vector<int> idWanted(int team,int phase);

		int correspondingId(std::vector<int> allPossibleId,std::vector<int> arTagId,std::vector<float> arTagDistance);

		int asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k);

};


#endif
