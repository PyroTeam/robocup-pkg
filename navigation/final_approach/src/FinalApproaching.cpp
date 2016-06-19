#include "final_approach/FinalApproaching.h"

#include "final_approach/Bumperlistener.h"
#include "final_approach/fa_utils.h"
#include "final_approach/LaserScan.h"
#include "final_approach/Point.h"
#include "final_approach/OdomFA.h"
#include "final_approach/Sharps.h"
#include "final_approach/GameStateFA.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <cmath>
#include <vector>
#include <list>

FinalApproaching::~FinalApproaching(void)
{
}

void FinalApproaching::preemptCB()
{
	ROS_INFO("%s: Preempted", m_actionName.c_str());
	m_as.setPreempted();
}

void FinalApproaching::executeCB(const final_approach_msg::FinalApproachingGoalConstPtr &goal)
{
	ros::Rate loopRate(g_loopFreq);
	bool firstTimeInLoop = false;  // Used to reduce logging, see ROS_DEBUG_COND(firstTimeInLoop, "[...]");

	// General initialization
	bool success = true;
	m_feedback.percent_complete = 0;
	m_type = goal->type;
	m_side = goal->side;
	m_parameter = goal->parameter;
	ROS_INFO("%s: Execute. Start FinalApproach sequence of type %i with side %i and parameter %i",
	         m_actionName.c_str(), m_type, m_side, m_parameter);
	if (m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT)
	{
		ROS_INFO("FinalApproach got LIGHT parameter, will skip laser regulation");
	}

	// BumperListener
	BumperListener bp;

	// Odom
	OdomFA odom;
	float initOrientation = odom.getOrientationZ();
	bool initOdom = false;

	// ARTag
	ArTagFA at;
	int arTagId_idx = -1;
	int locateArTagPhase = 0;
	int avancementArTag = 0;
	std::vector<int> id;
	std::vector<float> px = at.getPositionX();
	std::vector<float> pz = at.getPositionZ();
	std::vector<float> oz = at.getOrientationZ();
	std::vector<float> arTagDistance = at.getDistance();
	std::vector<arTag_t> arTags = at.getArTags();

	// Sharps
	Sharps sharps;
	bool obstacle = false;
	std::vector<bool> allObstacles = sharps.getObstacle();

	// GameState
	GameStateFA gameState;

	// LaserScan
	LaserScan ls;
	bool angleAsservDone = 0, yAsservDone = 0, xAsservDone = 0;
	int asservLaserYOk_cpt = 0;
	float positionY = 0, gradient = 0, ortho = 0, moyY = 0, moyO = 0;
	int asservLaser_cpt = 0;
	std::list<float> listPositionY, listOrtho;
	int nbAsservLaserYOkNeeded = 20;

	// Reload parameters
	refreshParams();

	// Wait for sensors
	ROS_INFO("Wait sensors infos");
	firstTimeInLoop = true;
	while (ros::ok() && !bp.getState())
	{
		ROS_INFO_COND(firstTimeInLoop, "Wait sensors infos - process");
		firstTimeInLoop = false;

		// Make sure that the action hasn't been canceled
		if (!m_as.isActive())
		{
			ROS_INFO("Action canceled during wait infos looop");
			stopRobot();
			return;
		}

		if (odom.getTurn())  // Make sure mandatory data are received at least once
			break;

		loopRate.sleep();
	}
	if (!firstTimeInLoop)
		ROS_INFO("Wait sensors infos - DONE");
	else
		ROS_WARN("Wait sensors infos - SKIPPED");










	// Try to locate a correct ARTag
	ROS_INFO("Locate ArTag");
	firstTimeInLoop = true;
	// TODO: quickfix idWanted
	std::vector<int> allPossibleId = idWanted(1 /*gameState.getPhase()*/);
	while (ros::ok() && !bp.getState() && arTagId_idx == -1 && locateArTagPhase != 3)
	{
		ROS_INFO_COND(firstTimeInLoop, "Locate ArTag - process");
		firstTimeInLoop = false;

		// Make sure that the action hasn't been canceled
		if (!m_as.isActive())
		{
			ROS_INFO("Action canceled during ArTag search looop");
			stopRobot();
			return;
		}

		if (at.hasArTags())
		{
			// TODO: Make sure it works
			arTagId_idx = correspondingId(allPossibleId, at.getId(), at.getDistance());
		}

		if (locateArTagPhase == 0)
		{
			initOrientation = odom.getOrientationZ();
			locateArTagPhase++;
		}

		m_msgTwist.angular.z = cameraScanVelocity(locateArTagPhase);
		float newOrientation = odom.getOrientationZ() - initOrientation;
		locateArTagPhase = phaseDependingOnOrientation(newOrientation, locateArTagPhase);

		m_pubMvt.publish(m_msgTwist);
		loopRate.sleep();
	}
	if (!firstTimeInLoop)
		ROS_INFO("Locate ArTag - DONE");
	else
		ROS_WARN("Locate ArTag - SKIPPED");










	// Asservissement ARTag camera
	ROS_INFO("ArTag Asservissement");
	firstTimeInLoop = true;
	while (ros::ok() && !bp.getState() && locateArTagPhase != 3 && avancementArTag == 0 && obstacle == false)
	{
		ROS_INFO_COND(firstTimeInLoop, "ArTag Asservissement - process");
		firstTimeInLoop = false;

		// Make sure that the action hasn't been canceled
		if (!m_as.isActive())
		{
			ROS_INFO("Action canceled during ArTag asserv looop");
			stopRobot();
			return;
		}

		// If at least one ARTag found
		if (at.hasArTags())
		{
			arTagId_idx = correspondingId(allPossibleId, at.getArTags());
			if (arTagId_idx != -1)
			{
				avancementArTag = FinalApproaching::asservissementCameraNew(at.getArTags()[arTagId_idx]);
			}
			else
			{
				ROS_WARN_THROTTLE(1.0, "NO Wanted ArTag found. Unable to do camera approach");
			}
		}
		allObstacles = sharps.getObstacle();
		obstacle = false;  // obstacleDetection(allObstacles, k, oz, pz);

		loopRate.sleep();
	}
	if (!firstTimeInLoop)
		ROS_INFO("ArTag Asservissement - DONE");
	else
	{
		ROS_WARN("ArTag Asservissement - SKIPPED");
		ROS_DEBUG_NAMED("investigation", "Bumper: %d | Phase: %d | AvancementArTag: %d | Obstacle: %d", bp.getState(),
		                locateArTagPhase, avancementArTag, obstacle);
	}










	// Asservissement laserScan
	ROS_INFO("LaserScan Asservissement");
	firstTimeInLoop = true;
	while (ros::ok() && !bp.getState() && locateArTagPhase != 3 && avancementArTag == 1 && !xAsservDone && obstacle == false
		&& m_parameter != final_approach_msg::FinalApproachingGoal::LIGHT)
	{
		ROS_INFO_COND(firstTimeInLoop, "LaserScan Asservissement - process");
		firstTimeInLoop = false;

		// Make sure that the action hasn't been canceled
		if (!m_as.isActive())
		{
			ROS_INFO("Action canceled during LaserScan asserv looop");
			stopRobot();
			return;
		}

		// Loop at loop beginning because it's easier
		loopRate.sleep();

		// If the scan isn't complete, skip
		if (ls.getRanges().empty())
		{
			ROS_INFO_ONCE("Waiting LaserScan data");
			continue;
		}

		ROS_INFO_ONCE("Got a complete LaserScan");

		std::vector<float> ranges = ls.getRanges();
		float angleMin 	= ls.getAngleMin();
		double angleInc = ls.getAngleInc();
		float rangeMin 	= ls.getRangeMin() - 0.10;  // Ignoring limit ranges is currently bugguy
		float rangeMax 	= ls.getRangeMax() + 0.10;  // Ignoring limit ranges is currently bugguy


		// Segmente le scan laser en plusieurs objets
		std::list<std::vector<Point> > listPointsVectors =
		    							objectsConstruction(ranges, angleMin, angleInc, rangeMin, rangeMax);

		// Récupère le segment ressemblant à une machine le plus proche
		Segment seg = segmentConstruction(listPointsVectors, ranges, angleMin, angleInc);

		if (!seg.isConstructed())
		{
			ROS_DEBUG("No segment found");
			continue;
		}

		// Publish segment marker for debug
		publishSegmentMarker(ls, seg);

		// XXX: Un moyennage (pas trop dégeu, sur base de repère robot, à grand renfor de tf) des informations d'entrée
		// pourra s'avérer utile. A voir.
		angleAsservDone = asservissementAngle(M_PI/2, seg.getAngle());


		if (angleAsservDone && (asservLaserYOk_cpt < nbAsservLaserYOkNeeded))
		{
			yAsservDone = asservissementPositionY(objectifY(), -seg.getMiddlePoint().y);
			if (yAsservDone)
			{
				asservLaserYOk_cpt++;
			}
		}

		if (angleAsservDone && asservLaserYOk_cpt >= nbAsservLaserYOkNeeded)
		{
			xAsservDone = asservissementPositionX(objectifX(), seg.distanceOrthoLaserSegment());
		}

		// TODO: Utiliser les plotData
		// TODO: Faire un bon feedback
		m_feedback.percent_complete = 33;
		m_as.publishFeedback(m_feedback);

		// TODO: S'assurer qu'on a un bon pattern d'execution de ros node
		// (le ros::spinonce dans une boucle dans une callback, p-e pas l'idéal)
		ros::spinOnce();


		loopRate.sleep();
	}
	if (!firstTimeInLoop)
		ROS_INFO("LaserScan Asservissement - DONE");
	else
		ROS_WARN("LaserScan Asservissement - SKIPPED");










	// Reinit
	stopRobot();

	// If any problem
	if (bp.getState() || locateArTagPhase == 3 || obstacle == true)
	{
		ROS_WARN("FinalApproach ended with some failures\n");

		if (bp.getState())
		{
			ROS_WARN("Failure : contact with obstacle\n");
			m_result.state = 1;
		}

		if (locateArTagPhase == 3)
		{
			ROS_WARN("Failure : no arTag found\n");
			m_result.state = 3;
		}

		if (obstacle == true)
		{
			ROS_WARN("Failure : an obstacle is too near\n");
			m_result.state = 2;
		}

		success = false;
	}

	if (success)
	{
		m_result.success = true;
		m_result.state = 0;
		ROS_INFO("%s: Succeeded", m_actionName.c_str());
		m_as.setSucceeded(m_result);
	}
	else
	{
		m_result.success = false;
		ROS_INFO("%s: Aborted", m_actionName.c_str());
		m_as.setAborted(m_result);
	}

	// Affiche l'erreur de l'approche finale
	debugFinalApproachResult(odom);
}

int FinalApproaching::avancement(int a, int b, int c)
{
	int tmp = -20;
	if (c == 2)
	{
		tmp = 100;
	}
	else
	{
		if (b == 1 && (c == 0 || c == 1))
		{
			tmp = 67;
		}
		else
		{
			if (a == 1 && b == 0 && (c == 0 || c == 1))
			{
				tmp = 33;
			}
			else
			{
				tmp = 0;
			}
		}
	}
	return tmp;
}

float FinalApproaching::objectifY()
{
	/**
	 * Les positions des objectifs sont données pour le côté INPUT d'une machine, y positif à gauche de la machine
	 */
	switch (m_parameter)
	{
		case final_approach_msg::FinalApproachingGoal::S1:
			return m_yPoseS1();
		case final_approach_msg::FinalApproachingGoal::S2:
			return m_yPoseS2();
		case final_approach_msg::FinalApproachingGoal::S3:
			return m_yPoseS3();
		case final_approach_msg::FinalApproachingGoal::LANE_RS:
			return m_yPoseLANE_RS();
		case final_approach_msg::FinalApproachingGoal::LIGHT:
		case final_approach_msg::FinalApproachingGoal::LIGHT_OLD:
			return m_yPoseLIGHT_OLD();
		case final_approach_msg::FinalApproachingGoal::CONVEYOR:
		// Le convoyeur est décentré, selon le côté il sera plutôt à gauche ou plutôt à droite
			if (m_side == final_approach_msg::FinalApproachingGoal::IN)
				return m_yPoseCONVEYOR();
			return -m_yPoseCONVEYOR();
		default:
			ROS_ERROR_THROTTLE(3.0, "Unknown parameter %d", m_parameter);
			return 42;
	}
}

float FinalApproaching::objectifX()
{
	// XXX: Paramétrer selon repère robot, voir repère préhenseur
	if (	m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT
		||  m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT_OLD)
	{
		return 0.35;
	}
	return 0.16;
}

std::list<std::vector<Point> > FinalApproaching::objectsConstruction(std::vector<float> ranges, float angleMin,
                                                                     double angleInc, float rangeMin, float rangeMax,
                                                                     float margin)
{
	assert(rangeMin <= rangeMax);

	// Init the list of points' vectors
	std::list<std::vector<Point> > listPointsVectors;
	Point pointInit(ranges[0], angleMin);
	std::vector<Point> pointsVectorInit;
	pointsVectorInit.push_back(pointInit);
	listPointsVectors.push_back(pointsVectorInit);

	std::list<std::vector<Point> >::iterator pointsVector_it = listPointsVectors.begin();

	for (int i = 1; i < ranges.size(); i++)
	{
		if ((ranges[i] > rangeMin) && (ranges[i] < rangeMax))
		{
			Point point(ranges[i], angleMin + (float)i * angleInc);

			// If nearby points (less than margin (0.05 meters recommended)
			if (std::abs(ranges[i] - ranges[i - 1]) < margin)
			{
				pointsVector_it->push_back(point);
			}
			else
			{
				// New object
				std::vector<Point> pointsVector;
				pointsVector.push_back(point);
				listPointsVectors.push_back(pointsVector);
				pointsVector_it++;
			}
		}
	}

	ROS_DEBUG_NAMED("objectsConstruction", "Nombre d elements de listPointsVectors: %d", (int)listPointsVectors.size());
	int j = 0, min = 0;
	for (pointsVector_it = listPointsVectors.begin(); pointsVector_it != listPointsVectors.end(); pointsVector_it++)
	{
		ROS_DEBUG_NAMED("objectsConstruction", "Nombre de points de l'objet #%d: %zu", j, pointsVector_it->size());
		ROS_DEBUG_NAMED("objectsConstruction", "startId: %d | stopID: %lu", min, min + pointsVector_it->size());
		min += pointsVector_it->size();
		j++;
	}

	return listPointsVectors;
}

Segment FinalApproaching::segmentConstruction(std::list<std::vector<Point> > listPointsVectors,
                                                            std::vector<float> ranges, float angleMin, double angleInc)
{
	std::vector<Segment> tabSegments;
	std::list<std::vector<Point> >::iterator pointsVector_it;
	int rangesStart_idx = 0, object_idx = 0;

	for (pointsVector_it = listPointsVectors.begin(); pointsVector_it != listPointsVectors.end(); pointsVector_it++)
	{
		float objLength;
		Point pmin(0, 0);
		Point pmax(0, 0);
		size_t lowerLimit_idx = rangesStart_idx;
		size_t upperLimit_idx = rangesStart_idx + pointsVector_it->size() - 1;

		assert(ranges.size() >= lowerLimit_idx);
		assert(lowerLimit_idx >= 0);
		assert(ranges.size() >= upperLimit_idx);
		assert(upperLimit_idx >= 0);

		// Not enough points in object
		if (pointsVector_it->size() <= 1)
		{
			rangesStart_idx += pointsVector_it->size();
			object_idx++;
			continue;
		}

		// Init pmin and pmax
		pmin.setR(ranges[lowerLimit_idx]);
		pmin.setPhi(angleMin + (double)lowerLimit_idx * angleInc);

		pmax.setR(ranges[upperLimit_idx]);
		pmax.setPhi(angleMin + (double)upperLimit_idx * angleInc);

		objLength = objectLengthNew(*pointsVector_it);

		// TODO: Refacto
		// Ce que je comprend ici :
		//  * Un segment est construit par regression linéaire sur tous les points de l'objet
		//  * Des informations supplémentaires sont settées (utiles ou pas ?)
		//  * Le segment le plus proche est conservé ...

		// If object length around 70 cm
		if (objLength > 0.65 && objLength < 0.75)
		{
			// Construction of segment
			Segment segm;
			segm.linearRegression(*pointsVector_it);

			tabSegments.push_back(segm);
		}

		rangesStart_idx += pointsVector_it->size();
		object_idx++;
	}

	// If no segment founds, return an empty one
	if (tabSegments.empty())
	{
		Segment emptySeg;
		return emptySeg;
	}

	// Only the nearest segment is kept
	return tabSegments[nearestSegment(tabSegments)];
}

float FinalApproaching::objectLength(int i, int j, std::list<std::vector<Point> > listPointsVectors,
                                     std::vector<float> ranges, float angleMin, double angleInc)
{
	std::list<std::vector<Point> >::iterator it = listPointsVectors.begin();
	int compteur = 0;
	int cpt2 = 0;
	while (compteur != i)
	{
		compteur++;
		cpt2 = cpt2 + it->size();
		it++;
	}

	// TODO: Check the validity of this condition
	if (it != listPointsVectors.end() && i != j)
	{
		std::vector<Point> pointsVector = *it;
		Point pmin(ranges[i], angleMin + (double)i * angleInc);
		Point pmax(ranges[j], angleMin + (double)j * angleInc);
		return distance2points(pointsVector[0], pointsVector[pointsVector.size() - 1]);
	}

	return 0;
}

float FinalApproaching::objectLengthNew(std::vector<Point> &pointsVector)
{
	return distance2points(pointsVector.front(), pointsVector.back());
}

int FinalApproaching::nearestSegment(std::vector<Segment> tabSegments)
{
	int nearest = 0;
	for (int i = 0; i < tabSegments.size(); i++)
	{
		if (tabSegments[i].distanceLaserSegment() < tabSegments[nearest].distanceLaserSegment())
		{
			nearest = i;
		}
	}
	return nearest;
}

// Condition préalable: la machine est a 90° du laser
float FinalApproaching::distanceOrtho(Segment s, std::vector<float> ranges, float angleMin, double angleInc)
{
	float ortho = 0.0;
	int min = s.getMinRanges();
	int max = s.getMaxRanges();

	assert(ranges.size() >= min);
	assert(min >= 0);
	assert(ranges.size() >= max);
	assert(max >= 0);

	ROS_DEBUG("distanceOrtho - range idx (min : max) -> (%d : %d)", min, max);
	ROS_DEBUG("distanceOrtho - range val (min : max) -> (%f : %f)", ranges[min], ranges[max]);
	if (max - min >= 9)
	{
		ROS_DEBUG(
		    "distanceOrtho - ranges val before max (max : max-9) -> (%f : %f : %f : %f : %f : %f : %f : %f : %f : %f)",
		    ranges[max - 0], ranges[max - 1], ranges[max - 2], ranges[max - 3], ranges[max - 4], ranges[max - 5],
		    ranges[max - 6], ranges[max - 7], ranges[max - 8], ranges[max - 9]);
		ROS_DEBUG(
		    "distanceOrtho - ranges val after min (min : min+9) -> (%f : %f : %f : %f : %f : %f : %f : %f : %f : %f)",
		    ranges[min + 0], ranges[min + 1], ranges[min + 2], ranges[min + 3], ranges[min + 4], ranges[min + 5],
		    ranges[min + 6], ranges[min + 7], ranges[min + 8], ranges[min + 9]);
		// TODO: REMOVE
		ROS_DEBUG(
		    "distanceOrtho - ranges val after max (max : max+9) -> (%f : %f : %f : %f : %f : %f : %f : %f : %f : %f)",
		    ranges[max + 0], ranges[max + 1], ranges[max + 2], ranges[max + 3], ranges[max + 4], ranges[max + 5],
		    ranges[max + 6], ranges[max + 7], ranges[max + 8], ranges[max + 9]);
		ROS_DEBUG(
		    "distanceOrtho - ranges val before min (min : min-9) -> (%f : %f : %f : %f : %f : %f : %f : %f : %f : %f)",
		    ranges[min - 0], ranges[min - 1], ranges[min - 2], ranges[min - 3], ranges[min - 4], ranges[min - 5],
		    ranges[min - 6], ranges[min - 7], ranges[min - 8], ranges[min - 9]);
	}

	Point gauche(ranges[max], angleMin + (double)max * angleInc);
	Point droite(ranges[min], angleMin + (double)min * angleInc);

	// Si le laser se trouve entre les deux points extremes du segment
	// Si yg et yd sont de signes différents
	ROS_DEBUG("distanceOrtho - pGauche (%f, %f)", gauche.getX(), gauche.getY());
	ROS_DEBUG("distanceOrtho - pDroit (%f, %f)", droite.getX(), droite.getY());
	// If the laser is between the two extremities of the machine
	if (gauche.getY() * droite.getY() < 0)
	{
		int tmp = min;
		for (int i = min; i <= max; i++)
		{
			if (ranges[tmp] > ranges[i])
			{
				tmp = i;
			}
		}
		ortho = ranges[tmp];
	}
	else
	{
		float orthoMin = ranges[min] * cos(angleMin + (double)min * angleInc);
		float orthoMax = ranges[max] * cos(angleMin + (double)max * angleInc);
		ortho = (orthoMin + orthoMax) / (float)2;
		ROS_INFO("orthoMin: %f orthoMax: %f", orthoMin, orthoMax);
	}
	return ortho;
}

float FinalApproaching::positionYLaser(Segment s, std::vector<float> ranges, float angleMin, double angleInc)
{
	/*
	int tmp = 0;
	int min = s.getMinRanges();
	int i = 0;
	int max=s.getMaxRanges();
	for(int i=min; i<max; i++)
	{
	    if(ranges[tmp] > ranges[i])
	    {
	        tmp = i;
	    }
	}
	float t = s.getDistance();
	float d = distanceOrtho(s,ranges,angleMin,angleInc);
	//to get the real extremity
	while(std::abs(ranges[tmp-i]-ranges[tmp-1-i])<0.03)
	{
	    ROS_ERROR_COND(ranges.size() <= tmp-i || tmp-i < 0 || ranges.size() <= tmp-i-1 || tmp-i-1 < 0, "OOPS, Bad index
	vector acess, after line %d in file %s", __LINE__, __FILE__);
	    i++;
	}
	float right = sqrt(ranges[tmp-i+1]*ranges[tmp-i+1] - d*d);
	i=0;
	//to get the other real extremity
	while(std::abs(ranges[tmp+i]-ranges[tmp+i+1])<0.03)
	{
	    ROS_ERROR_COND(ranges.size() <= tmp+i || tmp+i < 0 || ranges.size() <= tmp+i+1 || tmp+i+1 < 0, "OOPS, Bad index
	vector acess, after line %d in file %s", __LINE__, __FILE__);
	    i++;
	}
	float left = sqrt(ranges[tmp+i-1]*ranges[tmp+i-1] -d*d);
	ROS_DEBUG("taille: %f ortho: %f",t,d);
	ROS_DEBUG("right: %f t-left: %f",left,t-right);
	//do an average
	return (right+t-left)/(float)2;
	*/
	// float leftr = ranges[tmp+i-1];
	// float leftphi = (float)(tmp+i-1)*(angleMin+(float)(tmp+i-1)*angleInc);
	geometry_msgs::Point right = s.getMinPoint();
	geometry_msgs::Point left = s.getMaxPoint();
	float segmentSize = sqrt((left.x - right.x) * (left.x - right.x) + (left.y - right.y) * (left.y - right.y));
	ROS_DEBUG("left.x: %f left.y: %f right.x: %f right.y: %f segmentSize: %f", left.x, left.y, right.x, right.y,
	         segmentSize);
	return right.y;
	// return (left.y-segmentSize+right.y)/(float)2;
}

// XXX: A retirer / corriger. L'approche finale n'a pas à gérer ce genre de choses
std::vector<int> FinalApproaching::idWanted(int phase)
{
	std::vector<int> tabId;
	// Exploration phase
	if (phase == 0)
	{
		switch (m_teamColor)
		{
		case CYAN:
			tabId.push_back(C_CS1_OUT);
			tabId.push_back(C_CS2_OUT);
			tabId.push_back(C_RS1_OUT);
			tabId.push_back(C_RS2_OUT);
			tabId.push_back(C_BS_OUT);
			tabId.push_back(C_DS_IN);
			break;
		case MAGENTA:
			tabId.push_back(M_CS1_OUT);
			tabId.push_back(M_CS2_OUT);
			tabId.push_back(M_RS1_OUT);
			tabId.push_back(M_RS2_OUT);
			tabId.push_back(M_BS_OUT);
			tabId.push_back(M_DS_IN);
			break;
		}
	}
	// Production phase
	else
	{
		switch (m_teamColor)
		{
		// Cyan
		case CYAN:
			switch (m_type)
			{
			// BS
			case final_approach_msg::FinalApproachingGoal::BS:
				tabId.push_back(C_BS_IN);
				break;
			// RS
			case final_approach_msg::FinalApproachingGoal::RS:
				tabId.push_back(C_RS1_IN);
				tabId.push_back(C_RS2_IN);
				break;
			// CS
			case final_approach_msg::FinalApproachingGoal::CS:
				tabId.push_back(C_CS1_IN);
				tabId.push_back(C_CS2_IN);
				break;
			// DS
			case final_approach_msg::FinalApproachingGoal::DS:
				tabId.push_back(C_DS_IN);
				break;
			}
		// Magenta
		case MAGENTA:
			switch (m_type)
			{
			case final_approach_msg::FinalApproachingGoal::BS:
				tabId.push_back(M_BS_IN);
				break;
			case final_approach_msg::FinalApproachingGoal::RS:
				tabId.push_back(M_RS1_IN);
				tabId.push_back(M_RS2_IN);
				break;
			case final_approach_msg::FinalApproachingGoal::CS:
				tabId.push_back(M_CS1_IN);
				tabId.push_back(M_CS2_IN);
				break;
			case final_approach_msg::FinalApproachingGoal::DS:
				tabId.push_back(M_DS_IN);
				break;
			}
		}

		// Out
		if (m_side == final_approach_msg::FinalApproachingGoal::OUT)
		{
			for (int i = 0; i < tabId.size(); i++)
			{
				tabId[i]++;
			}
		}
	}

	return tabId;
}

// TODO: Remove is unused
int FinalApproaching::correspondingId(std::vector<int> allPossibleId, std::vector<int> arTagId,
                                      std::vector<float> arTagDistance)
{
	int correspondingId = -1;
	std::vector<int> ids;
	ids.clear();
	if (!arTagId.empty())
	{
		// oop to get the good ARTags
		for (int i = 0; i < allPossibleId.size(); i++)
		{
			for (int k = 0; k < arTagId.size(); k++)
			{
				ROS_DEBUG_NAMED("investigation", "Possible %d VS Found %d", allPossibleId[i], arTagId[k]);
				if (arTagId[k] == allPossibleId[i])
				{
					ids.push_back(k);
					ROS_DEBUG_NAMED("investigation", "Match");
				}
			}
		}

		if (!ids.empty() && !arTagDistance.empty())
		{
			int tmp = ids[0];
			ROS_DEBUG("taille de ids: %d de arTagDistance: %d", (int)ids.size(), (int)arTagDistance.size());
			// loop to keep the nearest ARTag
			for (int j = 0; j < ids.size(); j++)
			{
				ROS_DEBUG("ids[j]: %d tmp: %d", ids[j], tmp);
				if (arTagDistance[ids[j]] < arTagDistance[tmp])
				{
					tmp = ids[j];
				}
			}
			correspondingId = tmp;
		}
	}
	return correspondingId;
}

int FinalApproaching::correspondingId(std::vector<int> allPossibleId, std::vector<arTag_t> arTags)
{
	float tmpDist = FLT_MAX;
	int arTagIdx = -1;
	if (!arTags.empty())
	{
		for (int i = 0; i < allPossibleId.size(); i++)
		{
			for (int k = 0; k < arTags.size(); k++)
			{
				ROS_DEBUG_NAMED("investigation", "Possible %d VS Found %d", allPossibleId[i], arTags[k].id);
				if (allPossibleId[k] == arTags[k].id)
				{
					ROS_DEBUG_NAMED("investigation", "Match");
					if (arTags[k].distance < tmpDist)
					{
						tmpDist = arTags[k].distance;
						arTagIdx = k;
						ROS_DEBUG_NAMED("investigation", "New minimum: %f for id: %d", tmpDist, arTags[k].id);
					}
				}
			}
		}
	}

	return arTagIdx;
}

int FinalApproaching::asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k,
                                           ArTagFA &arTag)
{
	int avancementArTag = 0;

	if (!px.empty() && !pz.empty() && !oz.empty())
	{
		ROS_DEBUG_NAMED("investigation", "PX: %f, PZ: %f, OZ: %f", px[k], pz[k], oz[k]);
		if (std::abs(px[k]) < 0.005)
		{
			m_msgTwist.linear.y = 0;
		}
		else
		{
			m_msgTwist.linear.y = -0.75 * px[k];
		}

		if (std::abs(pz[k] - 0.50) < 0.01)
		{
			m_msgTwist.linear.x = 0;
		}
		else
		{
			m_msgTwist.linear.x = 0.25 * (pz[k] - 0.50);
		}

		if (std::abs(oz[k]) < 0.01)
		{
			m_msgTwist.angular.z = 0;
		}
		else
		{
			m_msgTwist.angular.z = 0.125 * oz[k];
		}

		if (m_msgTwist.linear.x == 0 && m_msgTwist.linear.y == 0 && m_msgTwist.angular.z == 0)
		{
			avancementArTag = 1;
		}
	}
	else
	{
		m_msgTwist.linear.x = 0;
		m_msgTwist.linear.y = 0;
		m_msgTwist.angular.z = 0;
	}

	m_pubMvt.publish(m_msgTwist);
	return avancementArTag;
}

// TODO: Regler et paramétrer cette phase
bool FinalApproaching::asservissementCameraNew(const arTag_t &target)
{
	// XXX: La fonction peut-elle être appelé sans arTag valide ? A vérifier


	bool finished = true;

	float errX = target.pose.position.z;  // A corriger une fois les transformations appliquée
	float errY = -target.pose.position.x;  // A corriger une fois les transformations appliquée
	float errYaw = target.yaw;  // A corriger une fois les transformations appliquée

	ROS_DEBUG_NAMED("artag", "asservissementCameraNew - errX: %f | errY: %f | errYaw: %f", errX, errY, errYaw);

	if (m_parameter != final_approach_msg::FinalApproachingGoal::LIGHT)
	{
		// Asserv en Y
		if (std::abs(errY) < 0.01)	// 1cm
		{
			m_msgTwist.linear.y = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.y = 0.75 * errY;
		}

		// Asserv en X
		if (std::abs(errX - 0.50) < 0.01) // 1cm
		{
			m_msgTwist.linear.x = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.x = 0.75 * (errX - 0.50);
		}

		// Asserv en angle
		if (std::abs(errYaw) < 0.01) // 0.01 rad -> 0.5 deg
		{
			m_msgTwist.angular.z = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.angular.z = 0.5 * errYaw;
		}
	}
	else
	{
		// Asserv en Y
		if (std::abs(errY) < 0.03)	// 3cm
		{
			m_msgTwist.linear.y = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.y = 0.75 * errY;
		}

		// Asserv en X
		if (std::abs(errX - 0.50) < 0.03) // 3cm
		{
			m_msgTwist.linear.x = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.x = 0.75 * (errX - 0.50);
		}

		// Asserv en angle
		if (std::abs(errYaw) < 0.02) // 0.02 rad -> 1 deg
		{
			m_msgTwist.angular.z = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.angular.z = 0.5 * errYaw;
		}
	}

	m_pubMvt.publish(m_msgTwist);
	return finished;
}

float FinalApproaching::cameraScanVelocity(int phase)
{
	float tmp;
	switch (phase)
	{
	case 1:
		// Turn on the left
		tmp = 0.25;
		break;
	case 2:
		// Turn on the right
		tmp = -0.25;
		break;
	case 3:
		tmp = 0;
		break;
	}
	return tmp;
}

int FinalApproaching::phaseDependingOnOrientation(float newOrientation, int phase)
{
	// To have newOrientation between -PI and PI
	if (newOrientation < -M_PI)
	{
		newOrientation = newOrientation + 2 * M_PI;
	}
	if (newOrientation > M_PI)
	{
		newOrientation = newOrientation - 2 * M_PI;
	}
	// If robot have finished turning left
	if (newOrientation > M_PI_2 && phase == 1)
	{
		phase = 2;
	}
	// If robot have finished turning right
	if (newOrientation < -M_PI_2 && phase == 2)
	{
		phase = 3;
	}
	return phase;
}

bool FinalApproaching::obstacleDetection(std::vector<bool> allObstacles, int k, std::vector<float> oz,
                                         std::vector<float> pz)
{
	bool obstacle = false;
	if (k != -1 && oz.size() > 0 && pz.size() > 0)
	{
		// probably wheels of the machine
		if (std::abs(oz[k]) > 0.45)
		{
			allObstacles[0] = false;
			allObstacles[1] = false;
			allObstacles[8] = false;
		}
		if (pz[k] < 0.55)
		{
			allObstacles[0] = false;
		}
	}
	for (int i = 0; i < allObstacles.size(); i++)
	{
		if (allObstacles[i] == true)
		{
			obstacle = true;
		}
	}
	return obstacle;
}


void FinalApproaching::stopRobot(void)
{
	geometry_msgs::Twist stop_msg;

	stop_msg.linear.x = 0;
	stop_msg.linear.y = 0;
	stop_msg.linear.z = 0;
	stop_msg.angular.x = 0;
	stop_msg.angular.y = 0;
	stop_msg.angular.z = 0;
	m_pubMvt.publish(stop_msg);

	return;
}



bool FinalApproaching::asservissementAngle(float setpoint, float measure)
{
	float err = setpoint - measure;
	m_plotData.angleErr = std::abs(err);

	m_msgTwist.angular.x = 0;
	m_msgTwist.angular.y = 0;
	m_msgTwist.angular.z = 0;

	// TODO: Changer de repère et retirer ce '-' sur la commande
	if(std::abs(err) < m_laserYawPidThreshold())
	{
		m_msgTwist.angular.z = 0;
		m_plotData.angleCmd = -m_msgTwist.angular.z;
		m_pubMvt.publish(m_msgTwist);
		return true;
	}
	else
	{
		m_msgTwist.angular.z = -m_laserYawPid.update(err);
		m_plotData.angleCmd = m_msgTwist.angular.z;
		m_pubMvt.publish(m_msgTwist);
		return false;
	}
}


bool FinalApproaching::asservissementPositionY(float setpoint, float measure)
{
	float err = setpoint - measure;
	m_plotData.YErr = std::abs(err);

	m_msgTwist.angular.x = 0;
	m_msgTwist.angular.y = 0;
	m_msgTwist.angular.z = 0;

	// TODO: Changer de repère et retirer ce '-' sur la commande
	if(std::abs(err) < m_laserYPidThreshold())
	{
		m_msgTwist.linear.y = 0;
		m_plotData.YCmd = m_msgTwist.linear.y;
		m_pubMvt.publish(m_msgTwist);
		return true;
	}
	else
	{
		m_msgTwist.linear.y = m_laserYPid.update(err);
		m_plotData.YCmd = m_msgTwist.linear.y;
		m_pubMvt.publish(m_msgTwist);
		return false;
	}
}


bool FinalApproaching::asservissementPositionX(float setpoint, float measure)
{
	float err = setpoint - measure;
	m_plotData.XErr = std::abs(err);

	m_msgTwist.angular.x = 0;
	m_msgTwist.angular.y = 0;
	m_msgTwist.angular.z = 0;

	// TODO: Changer de repère et retirer ce '-' sur la commande
	if(std::abs(err) < m_laserXPidThreshold())
	{
		m_msgTwist.linear.x = -m_laserXPid.update(err);
		m_plotData.XCmd = m_msgTwist.linear.x;
		m_pubMvt.publish(m_msgTwist);
		return true;
	}
	else
	{
		m_msgTwist.linear.x = -m_laserXPid.update(err);
		m_plotData.XCmd = m_msgTwist.linear.x;
		m_pubMvt.publish(m_msgTwist);
		return false;
	}
}

void FinalApproaching::publishSegmentMarker(LaserScan &ls, Segment &seg)
{
	// Debug Marker
	m_marker.header.frame_id = ls.getFrame();
	m_marker.header.stamp = ls.getStamp();

	m_marker.type = m_marker.LINE_LIST;
	m_marker.action = m_marker.ADD;
	m_marker.ns = ros::this_node::getName();
	m_marker.id = SEGMENT_MARKER_ID;
	m_marker.scale.x = 0.015;
	// RED
	m_marker.color.r = 1.0;
	m_marker.color.g = 0.0;
	m_marker.color.b = 0.0;
	m_marker.color.a = 1.0;

	m_marker.lifetime = ros::Duration(10);
	m_marker.frame_locked = false;

	// dY and dX
	geometry_msgs::Point middlePoint = seg.getMiddlePoint();
	float angle = seg.getAngle();
	float dY = sin(angle) * m_mpsWidth()/2;
	float dX = cos(angle) * m_mpsWidth()/2;
	ROS_DEBUG("Marker MPS is : (%f / %f) - %f rad", middlePoint.x, middlePoint.y, angle);

	// First point
	geometry_msgs::Point tmp_point;
	tmp_point.x = middlePoint.x + dX;
	tmp_point.y = middlePoint.y + dY;
	tmp_point.z = 0.05;
	m_marker.points.push_back(tmp_point);

	// Second point
	tmp_point.x = middlePoint.x - dX;
	tmp_point.y = middlePoint.y - dY;
	tmp_point.z = 0.05;
	m_marker.points.push_back(tmp_point);

	m_markerPub.publish(m_marker);
	m_marker.points.clear();
}

void FinalApproaching::debugFinalApproachResult(OdomFA &odom, float mpsX, float mpsY, float mpsTheta)
{
	float errX, errY, errYaw;

	float x = odom.getPositionX();
	float y = odom.getPositionY();
	float yaw = odom.getOrientationZ();
	float cosMps = std::cos(mpsTheta);
	float sinMps = std::sin(mpsTheta);

	// TODO: Inverser les signes de objectifY() si corrigé
	float goalY = mpsY - objectifX()*sinMps - objectifY()*cosMps;
	float goalX = mpsX - objectifX()*cosMps + objectifY()*sinMps;
	float goalYaw = mpsTheta;

	errX = std::abs(goalX - x) - 0.11;  // Prise en compte de la position laser
	errY = std::abs(goalY - y);
	errYaw = std::abs(goalYaw - yaw);

	ROS_INFO("FinalApproach result: goal (%5.3f | %5.3f) yaw: %3.1f", goalX, goalY, goalYaw*(180.0/M_PI));
	ROS_INFO("FinalApproach result: position (%5.3f | %5.3f) yaw: %3.1f", x, y, yaw*(180.0/M_PI));
	ROS_INFO("FinalApproach result: erreur (%5.3f cm| %5.3f cm) yaw: %3.1f", errX/100, errY/100, errYaw*(180.0/M_PI));
}
