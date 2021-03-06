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

	m_mode  = goal->mode;
	switch(m_mode)
	{
		case final_approach_msg::FinalApproachingGoal::CONTROLLED_BY_TOPIC:
			ROS_INFO("Controlled by topic FA requested");
			if (std::isnan(goal->control_param))
			{
				ROS_ERROR_STREAM("Controlled by topic FA requested without sending any setpoint on topic (topic:"<< m_control.getTopic() <<")");
				m_result.success = false;
				m_result.state = final_approach_msg::FinalApproachingResult::INVALID_ORDER;
				ROS_WARN("%s: Aborted. Invalid order.", m_actionName.c_str());
				m_as.setAborted();
				return;
			}

			m_Controlled = true;
		break;

		case final_approach_msg::FinalApproachingGoal::CONTROLLED_BY_PARAM:
			ROS_INFO("Controlled by param FA requested");
			m_Controlled = false;
			m_controlParamDist = goal->control_param;
		break;

		default:
			ROS_WARN("FA requested with invalid mode (mode:%d), will fallback on DEFAULT(mode:%d)"
				, goal->mode, final_approach_msg::FinalApproachingGoal::DEFAULT);
		case final_approach_msg::FinalApproachingGoal::DEFAULT:
			ROS_INFO("Standard FA requested");
			m_Controlled = false;
		break;
	}

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
	// max 0.5s, NB: g_loopFreq donne le nombre d'iterations en une seconde
	constexpr int threshCptLostArTags = g_loopFreq / 2;
	int cptLostArTags = 0;
	bool arTagDefinitelyLost = false;

	// Sharps
	Sharps sharps;
	bool obstacle = false;
	std::vector<bool> allObstacles = sharps.getObstacle();

	// GameState
	GameStateFA gameState;

	// LaserScan
	LaserScan ls;
	bool angleAsservDone = false, yAsservDone = false, xAsservDone = false;
	bool anglePhaseDone = false, yPhaseDone = false, laserAsservDone = false;
	int asservLaserYawOk_cpt = 0, asservLaserYOk_cpt = 0, asservLaserXOk_cpt = 0;
	float positionY = 0, gradient = 0, ortho = 0, moyY = 0, moyO = 0;
	std::list<float> listPositionY, listOrtho;

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


		// TODO: Utiliser les plotData
		// TODO: Faire un bon feedback
		m_feedback.percent_complete = 10;
		m_as.publishFeedback(m_feedback);

		// TODO: S'assurer qu'on a un bon pattern d'execution de ros node
		// (le ros::spinonce dans une boucle dans une callback, p-e pas l'idéal)
		ros::spinOnce();

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
	while (    ros::ok()
			&& !bp.getState()
			&& arTagId_idx == -1
			&& locateArTagPhase != 3
			&& !m_skipAsservCamera()
			&& !m_Controlled)
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


		// TODO: Utiliser les plotData
		// TODO: Faire un bon feedback
		m_feedback.percent_complete = 20;
		m_as.publishFeedback(m_feedback);

		// TODO: S'assurer qu'on a un bon pattern d'execution de ros node
		// (le ros::spinonce dans une boucle dans une callback, p-e pas l'idéal)
		ros::spinOnce();
		loopRate.sleep();
	}
	if (!firstTimeInLoop)
		ROS_INFO("Locate ArTag - DONE");
	else
		ROS_WARN("Locate ArTag - SKIPPED");









	// Asservissement ARTag camera
	ROS_INFO("ArTag Asservissement");
	firstTimeInLoop = true;
	while (    ros::ok()
			&& !bp.getState()
			&& locateArTagPhase != 3
			&& avancementArTag == 0
			&& !obstacle
			&& !m_skipAsservCamera()
			&& !m_Controlled
			&& !arTagDefinitelyLost)
	{
		ROS_INFO_COND(firstTimeInLoop, "ArTag Asservissement - process");
		firstTimeInLoop = false;
		std::vector<arTag_t> arTags_tmp = at.getArTags();

		// Make sure that the action hasn't been canceled
		if (!m_as.isActive())
		{
			ROS_INFO("Action canceled during ArTag asserv looop");
			stopRobot();
			return;
		}

		// Reset velocities
		m_msgTwist.linear.x = 0.0;
		m_msgTwist.linear.y = 0.0;
		m_msgTwist.linear.z = 0.0;
		m_msgTwist.angular.x = 0.0;
		m_msgTwist.angular.y = 0.0;
		m_msgTwist.angular.z = 0.0;

		// If at least one ARTag found
		if (!arTags_tmp.empty())
		{
			arTagId_idx = correspondingId(allPossibleId, arTags_tmp);
		}
		else
		{
			arTagId_idx = -1;
		}

		if (arTagId_idx != -1)
		{
			cptLostArTags = 0;
			avancementArTag = FinalApproaching::asservissementCameraNew(arTags_tmp[arTagId_idx]);
		}
		else
		{
			if (++cptLostArTags >= threshCptLostArTags)
			{
				ROS_ERROR("ArTag lost definitely, abort");
				arTagDefinitelyLost = true;
			}
			else
			{
				ROS_WARN("ArTag lost, will retry %d time%s", threshCptLostArTags-cptLostArTags
					, (threshCptLostArTags-cptLostArTags > 1)?"s":"");
			}
		}

		allObstacles = sharps.getObstacle();
		obstacle = false;  // obstacleDetection(allObstacles, k, oz, pz);


		// TODO: Utiliser les plotData
		// TODO: Faire un bon feedback
		m_feedback.percent_complete = 30;
		m_as.publishFeedback(m_feedback);

		// TODO: S'assurer qu'on a un bon pattern d'execution de ros node
		// (le ros::spinonce dans une boucle dans une callback, p-e pas l'idéal)
		ros::spinOnce();

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

	if (m_skipAsservCamera() || m_Controlled)
	{
		avancementArTag = 1;
	}










	// Asservissement laserScan
	ROS_INFO("LaserScan Asservissement");
	firstTimeInLoop = true;
	while (    ros::ok()
			&& !bp.getState()
			&& locateArTagPhase != 3
			&& avancementArTag == 1
			&& !laserAsservDone
			&& !obstacle
			&& m_parameter != final_approach_msg::FinalApproachingGoal::LIGHT
			&& !m_skipAsservLaser()
			&& !arTagDefinitelyLost)
	{
		ROS_INFO_COND(firstTimeInLoop, "LaserScan Asservissement - process");
		firstTimeInLoop = false;
		m_feedback.errorX = 0;
		m_feedback.errorY = 0;
		m_feedback.errorYaw = 0;

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

		// Reset velocities
		m_msgTwist.linear.x = 0.0;
		m_msgTwist.linear.y = 0.0;
		m_msgTwist.linear.z = 0.0;
		m_msgTwist.angular.x = 0.0;
		m_msgTwist.angular.y = 0.0;
		m_msgTwist.angular.z = 0.0;

		// XXX: Un moyennage (pas trop dégeu, sur base de repère robot, à grand renforts de tf) des informations d'entrée
		// pourra s'avérer utile. A voir.
		angleAsservDone = asservissementAngle(M_PI/2, seg.getAngle());
		asservLaserYawOk_cpt = angleAsservDone ? asservLaserYawOk_cpt+1 : 0;
		if (asservLaserYawOk_cpt >= m_laserYawPidNbSuccessNeeded())
		{
			anglePhaseDone = true;
		}


		if (anglePhaseDone)
		{
			ROS_WARN("Y - OBJ: %f, MEAS: %f", objectifY(), -seg.getMiddlePoint().y);
			yAsservDone = asservissementPositionY(objectifY(), -seg.getMiddlePoint().y);
			if (asservLaserYOk_cpt >= m_laserYPidNbSuccessNeeded())
			{
				yPhaseDone = true;
			}
			asservLaserYOk_cpt = yAsservDone ? asservLaserYOk_cpt+1 : 0;

		}

		if (anglePhaseDone && yPhaseDone)
		{
			ROS_WARN("X - OBJ: %f, MEAS: %f", objectifX(), -seg.distanceOrthoLaserSegment());
			xAsservDone = asservissementPositionX(objectifX(), -seg.distanceOrthoLaserSegment());
			asservLaserXOk_cpt = xAsservDone ? asservLaserXOk_cpt+1 : 0;
		}

		laserAsservDone = (	   asservLaserYawOk_cpt >= m_laserYawPidNbSuccessNeeded()
							&& asservLaserYOk_cpt >= m_laserYPidNbSuccessNeeded()
							&& asservLaserXOk_cpt >= m_laserXPidNbSuccessNeeded());

		// TODO: Utiliser les plotData
		// TODO: Faire un bon feedback
		m_feedback.percent_complete = 40;
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

	// TODO: Utiliser les plotData
	// TODO: Faire un bon feedback
	m_feedback.percent_complete = 100;
	m_as.publishFeedback(m_feedback);

	// If any problem
	if (bp.getState() || locateArTagPhase == 3 || obstacle || arTagDefinitelyLost)
	{
		ROS_WARN("FinalApproach ended with some failures\n");

		if (bp.getState())
		{
			ROS_WARN("Failure : contact with obstacle\n");
			m_result.state = final_approach_msg::FinalApproachingResult::OBSTACLE_HIT;
		}

		if (locateArTagPhase == 3)
		{
			ROS_WARN("Failure : no arTag found\n");
			m_result.state = final_approach_msg::FinalApproachingResult::COMPLETE_SCAN;
		}

		if (obstacle)
		{
			ROS_WARN("Failure : an obstacle is too near\n");
			m_result.state = final_approach_msg::FinalApproachingResult::OBSTACLE_NEAR;
		}

		if (arTagDefinitelyLost)
		{
			m_result.state = final_approach_msg::FinalApproachingResult::ARTAG_LOST;
		}

		success = false;
	}

	if (success)
	{
		m_result.success = true;
		m_result.state = final_approach_msg::FinalApproachingResult::UNKNOWN;
		ROS_INFO("%s: Succeeded", m_actionName.c_str());
		m_as.setSucceeded(m_result);
	}
	else
	{
		m_result.success = false;
		ROS_WARN("%s: Aborted", m_actionName.c_str());
		m_as.setAborted(m_result);
	}

	// Affiche l'erreur de l'approche finale
	debugFinalApproachResult(odom);
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
	if (m_Controlled)
	{
		// TODO: Utiliser TF et la connaissance de la géométrie du robot pour que ce paramètre contrôle bien la distance
		// gripper <-> machine
		if (m_mode == final_approach_msg::FinalApproachingGoal::CONTROLLED_BY_PARAM)
		{
			return m_controlParamDist + m_xPoseCONVEYOR();
		}
		else if (m_mode == final_approach_msg::FinalApproachingGoal::CONTROLLED_BY_TOPIC)
		{
			return m_controlTopicDist + m_xPoseCONVEYOR();
		}
		else
		{
			ROS_WARN_THROTTLE(1.0, "FA is in controlled mode, but actual mode (mode:%d) is unknowm. Will fallback on 10cm distance", m_mode);
			return -10;
		}
	}

	// On travaille avec un repère machine, orienté comme le repère LASER
	// à -0.35 le laser se trouve à 35cm de la machine, à -0.16, à 16cm (soit au contact)
	if (	m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT
		||  m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT_OLD)
	{
		return m_xPoseLIGHT_OLD();
	}
	return m_xPoseCONVEYOR();
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
	constexpr float mpsWidthRelativeMargin = 0.10;
	float mpsWidth = m_mpsWidth();
	float mpsWidthMin = mpsWidth * (1 - mpsWidthRelativeMargin);
	float mpsWidthMax = mpsWidth * (1 + mpsWidthRelativeMargin);
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

		ROS_DEBUG_NAMED("segmentConstruction", "Object #%ld length: %f"
			, std::distance(listPointsVectors.begin(), pointsVector_it), objLength);
		ROS_DEBUG_NAMED("segmentConstruction", "Min:%f Max:%f"
			, mpsWidthMin, mpsWidthMax);

		// If object length around 70 cm
		if (objLength > mpsWidthMin && objLength < mpsWidthMax)
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

// TODO: Remove if unused
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
				if (allPossibleId[i] == arTags[k].id)
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

// TODO: Regler et paramétrer cette phase
bool FinalApproaching::asservissementCameraNew(const arTag_t &target)
{
	constexpr float xDist = 0.50;
	constexpr float linearKp = 1.0;
	constexpr float angularKp = 0.50;
	constexpr float absYOffset = 0.025;
	float yOffset;
	if (m_side == final_approach_msg::FinalApproachingGoal::IN)
	{
		yOffset = -absYOffset;
	}
	else
	{
		yOffset = +absYOffset;
	}

	// XXX: La fonction peut-elle être appelé sans arTag valide ? A vérifier


	bool finished = true;
	int xSuccessJar = m_camXPidNbSuccessNeeded();
	int ySuccessJar = m_camYPidNbSuccessNeeded();
	int yawSuccessJar = m_camYawPidNbSuccessNeeded();

	float errX = target.pose.position.z - xDist;  // A corriger une fois les transformations appliquée
	float errY = yOffset-target.pose.position.x;  // A corriger une fois les transformations appliquée
	float errYaw = target.yaw;  // A corriger une fois les transformations appliquée

	m_feedback.errorX = errX;
	m_feedback.errorY = errY;
	m_feedback.errorYaw = errYaw;

	ROS_DEBUG_NAMED("artag", "asservissementCameraNew - errX: %f | errY: %f | errYaw: %f", errX, errY, errYaw);

	if (m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT)
	{
		// Asserv en Y
		if (std::abs(errY) < m_camYPidThreshold())	// 0.5cm
		{
			m_msgTwist.linear.y = 0;
			if (ySuccessJar > 0)
			{
				ySuccessJar--;
			}
		}
		else
		{
			m_msgTwist.linear.y = m_camYPidKp() * errY;
			ySuccessJar = m_camYPidNbSuccessNeeded();
		}

		// Asserv en X
		if (std::abs(errX) < m_camXPidThreshold()) // 0.5cm
		{
			m_msgTwist.linear.x = 0;
			if (xSuccessJar > 0)
			{
				xSuccessJar--;
			}
		}
		else
		{
			m_msgTwist.linear.x = m_camXPidKp() * errX;
			xSuccessJar = m_camXPidNbSuccessNeeded();
		}

		// Asserv en angle
		if (std::abs(errYaw) < m_camYawPidThreshold()) // 0.01 rad -> 0.5 deg
		{
			m_msgTwist.angular.z = 0;
			if (yawSuccessJar > 0)
			{
				yawSuccessJar--;
			}
		}
		else
		{
			m_msgTwist.angular.z = m_camYawPidKp() * errYaw;
			yawSuccessJar = m_camYawPidNbSuccessNeeded();
		}

		finished =(xSuccessJar + ySuccessJar + yawSuccessJar) == 0;
	}
	else
	{
		constexpr float linearThreshold = 0.03;
		constexpr float angularThreshold = 0.02;

		// Asserv en Y
		if (std::abs(errY) < linearThreshold)	// 3cm
		{
			m_msgTwist.linear.y = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.y = linearKp * errY;
		}

		// Asserv en X
		if (std::abs(errX) < linearThreshold) // 3cm
		{
			m_msgTwist.linear.x = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.linear.x = linearKp * errX;
		}

		// Asserv en angle
		if (std::abs(errYaw) < angularThreshold) // 0.02 rad -> 1 deg
		{
			m_msgTwist.angular.z = 0;
		}
		else
		{
			finished = false;
			m_msgTwist.angular.z = angularKp * errYaw;
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

	if (newOrientation > M_PI/6 && phase == 1)
	{
		phase = 2;
	}
	// If robot have finished turning right
	if (newOrientation < -M_PI/6.0 && phase == 2)
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
		if (allObstacles[i])
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
	m_feedback.errorYaw = err;

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
	m_feedback.errorY = err;

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
	m_feedback.errorX = err;
	// TODO: Utiliser TF et la connaissance de la géométrie du robot pour que ce feedback représente bien la distance
	// gripper <-> machine
	m_feedback.distance_to_machine = err;

	m_msgTwist.angular.x = 0;
	m_msgTwist.angular.y = 0;
	m_msgTwist.angular.z = 0;

	if(std::abs(err) < m_laserXPidThreshold())
	{
		m_msgTwist.linear.x = m_laserXPid.update(err);
		m_plotData.XCmd = m_msgTwist.linear.x;
		m_pubMvt.publish(m_msgTwist);
		return true;
	}
	else
	{
		m_msgTwist.linear.x = m_laserXPid.update(err);
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

	// Middle point
	visualization_msgs::Marker mid_point_marker;
	mid_point_marker.header.frame_id = ls.getFrame();
	mid_point_marker.header.stamp = ls.getStamp();

	mid_point_marker.type = mid_point_marker.SPHERE;
	mid_point_marker.action = mid_point_marker.ADD;
	mid_point_marker.ns = ros::this_node::getName();
	mid_point_marker.id = MID_POINT_ID;
	mid_point_marker.scale.x = 0.03;
	mid_point_marker.scale.y = 0.03;
	mid_point_marker.scale.z = 0.03;
	// // BLUE
	mid_point_marker.color.r = 0.0;
	mid_point_marker.color.g = 0.0;
	mid_point_marker.color.b = 1.0;
	mid_point_marker.color.a = 1.0;

	mid_point_marker.lifetime = ros::Duration(10);
	mid_point_marker.frame_locked = false;

	mid_point_marker.pose.position.x = middlePoint.x;
	mid_point_marker.pose.position.y = middlePoint.y;
	mid_point_marker.pose.position.z = 0.05;

	mid_point_marker.pose.orientation.x = 0.0;
	mid_point_marker.pose.orientation.y = 0.0;
	mid_point_marker.pose.orientation.z = 0.0;
	mid_point_marker.pose.orientation.w = 1.0;

	m_markerPub.publish(mid_point_marker);
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
