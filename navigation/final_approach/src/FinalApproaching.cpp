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
	int angleAsservState = 0, yAsservState = 0, xAsservState = 0, asservLaserYOk_cpt = 0;
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
			px = at.getPositionX();
			pz = at.getPositionZ();
			oz = at.getOrientationZ();
			arTagId_idx = correspondingId(allPossibleId, at.getId(), at.getDistance());
			ROS_DEBUG("taille de px: %d de pz: %d de oz: %d et valeur de k: %d", (int)px.size(), (int)pz.size(),
			          (int)oz.size(), arTagId_idx);
			avancementArTag = FinalApproaching::asservissementCameraNew(at.getArTags()[arTagId_idx]);
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
	while (ros::ok() && !bp.getState() && locateArTagPhase != 3 && avancementArTag == 1 && xAsservState != 2 && obstacle == false)
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

		// If the scan is complete
		if (!ls.getRanges().empty())
		{
			ROS_INFO_ONCE("Got a complete LaserScan");
			std::vector<float> ranges = ls.getRanges();
			float angleMin = ls.getAngleMin();
			double angleInc = ls.getAngleInc();
			float rangeMin = ls.getRangeMin() - 0.10;  // Ignoring limit ranges is currently bugguy
			float rangeMax = ls.getRangeMax() + 0.10;  // Ignoring limit ranges is currently bugguy
			std::list<std::vector<Point> > listPointsVectors =
			    objectsConstruction(ranges, angleMin, angleInc, rangeMin, rangeMax);
			std::vector<Segment> tabSegments = segmentsConstruction(listPointsVectors, ranges, angleMin, angleInc);
			// At least one segment found
			if (tabSegments.size() > 0)
			{
				Segment seg = tabSegments[0];
				ROS_DEBUG("min seg: %d max seg %d", seg.getMinRanges(), seg.getMaxRanges());
				// If no error about the laserscan data
				if (seg.getMinRanges() >= 0 && seg.getMinRanges() <= ranges.size() && seg.getMaxRanges() >= 0 &&
				    seg.getMaxRanges() <= ranges.size())
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

					// First point
					geometry_msgs::Point tmp_point;
					tmp_point.x = seg.getMin().getX();
					tmp_point.y = seg.getMin().getY();
					tmp_point.z = 0.05;
					m_marker.points.push_back(tmp_point);

					// Second point
					tmp_point.x = seg.getMax().getX();
					tmp_point.y = seg.getMax().getY();
					tmp_point.z = 0.05;
					m_marker.points.push_back(tmp_point);

					m_markerPub.publish(m_marker);
					m_marker.points.clear();

					gradient = seg.getGradient();
					ROS_DEBUG("gradient du segment le plus proche : %f", gradient);
					positionY = positionYLaser(seg, ranges, angleMin, angleInc);
					ROS_DEBUG("l objet se trouve a environ %f m du bord", positionY);
					ortho = distanceOrtho(seg, ranges, angleMin, angleInc);
					ROS_DEBUG("distance orthogonale laser-machine : %f", ortho);


					ROS_INFO("Assev Laser - nouvelle iteration");
					// To do an average
					if (asservLaser_cpt < 1)
					{
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						asservLaser_cpt++;
					}
					else
					{
						listPositionY.pop_front();
						listOrtho.pop_front();
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						moyY = moy(listPositionY);
						moyO = moy(listOrtho);
						// to be in front of the machine
						angleAsservState = asservissementAngle(m_plotData, m_pubMvt, gradient);
						int min = seg.getMinRanges();
						int max = seg.getMaxRanges();
						Point gauche(ranges[max], angleMin + (double)min * angleInc);
						Point droite(ranges[min], angleMin + (double)max * angleInc);
						m_plotData.XLeft = gauche.getX();
						m_plotData.YLeft = gauche.getY();
						m_plotData.XRight = droite.getX();
						m_plotData.YRight = droite.getY();
						if ((angleAsservState == 1) && (asservLaserYOk_cpt < nbAsservLaserYOkNeeded))
						{
							// to move on the Y axis of the robot
							yAsservState = asservissementPositionY(m_plotData, m_pubMvt, moyY, objectifY(), gauche.getY(),
							                            droite.getY());
							ROS_DEBUG("Position Y: %f", -positionY);
							ROS_DEBUG("Moyenne Y: %f", -moyY);
							ROS_DEBUG("Objectif Y: %f", objectifY());
							// listPositionY.clear();
							if (yAsservState == 1)
							{
								asservLaserYOk_cpt++;
							}
						}
						if (angleAsservState == 1 && asservLaserYOk_cpt >= nbAsservLaserYOkNeeded)
						{
							// To move on the X axis of the robot
							ROS_WARN_ONCE("Skip to AsservX");
							xAsservState = asservissementPositionX(m_plotData, m_pubMvt, ortho, objectifX());
							ROS_DEBUG("Position X: %f", ortho);
							ROS_DEBUG("Objectif X: %f", objectifX());
						}
					}
				}
				m_plot.publish(m_plotData);
				m_feedback.percent_complete = avancement(angleAsservState, yAsservState, xAsservState);
				m_as.publishFeedback(m_feedback);
				ros::spinOnce();
				ROS_DEBUG("BumperState : %d ", bp.getState());
			}
			else
			{
				ROS_INFO_ONCE("Waiting LaserScan data");
			}

			loopRate.sleep();
		}
	}
	if (!firstTimeInLoop)
		ROS_INFO("LaserScan Asservissement - DONE");
	else
		ROS_WARN("LaserScan Asservissement - SKIPPED");

	// Reinit
	angleAsservState = 0;
	yAsservState = 0;
	xAsservState = 0;
	asservLaserYOk_cpt = 0;
	asservLaser_cpt = 0;
	avancementArTag = 0;
	locateArTagPhase = 0;

	listPositionY.clear();
	listOrtho.clear();

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
	switch (m_parameter)
	{
		case final_approach_msg::FinalApproachingGoal::S1:
			return 0.28;
		case final_approach_msg::FinalApproachingGoal::S2:
			return 0.175;
		case final_approach_msg::FinalApproachingGoal::S3:
			return 0.08;
		case final_approach_msg::FinalApproachingGoal::LANE_RS:
			return 0.09;
		case final_approach_msg::FinalApproachingGoal::LIGHT:
			return 0.35;
		case final_approach_msg::FinalApproachingGoal::CONVEYOR:
		// TODO: Pourquoi deux mesures différentes ?
			if (m_side == final_approach_msg::FinalApproachingGoal::IN)
				return 0.37;
			return 0.315;
		default:
			ROS_ERROR("Unknown parameter %d", m_parameter);
			return -1;
	}
}

float FinalApproaching::objectifX()
{
	// XXX: Paramétrer selon repère robot, voir repère préhenseur
	if (m_parameter == final_approach_msg::FinalApproachingGoal::LIGHT)
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

	ROS_DEBUG("Nombre d elements de listPointsVectors: %d", (int)listPointsVectors.size());
	int j = 0, min = 0;
	for (pointsVector_it = listPointsVectors.begin(); pointsVector_it != listPointsVectors.end(); pointsVector_it++)
	{
		ROS_DEBUG("Nombre de points de l'objet #%d: %zu", j, pointsVector_it->size());
		ROS_DEBUG("startId: %d | stopID: %lu", min, min + pointsVector_it->size());
		min += pointsVector_it->size();
		j++;
	}

	return listPointsVectors;
}

std::vector<Segment> FinalApproaching::segmentsConstruction(std::list<std::vector<Point> > listPointsVectors,
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
			Segment segm(pmin, pmax, lowerLimit_idx, upperLimit_idx);

			ROS_DEBUG_NAMED("segmentsConstrutcion", "Distance objet #%d du laser: %f", object_idx, segm.distanceLaserSegment(ranges));
			ROS_DEBUG_NAMED("segmentsConstrutcion", "Distance ortho de l'objet #%d: %f", object_idx, distanceOrtho(segm, ranges, angleMin, angleInc));

			geometry_msgs::Pose2D p = segm.linearRegression(*pointsVector_it);
			geometry_msgs::Point orthoMin = orthoProjection(pmin, p);
			geometry_msgs::Point orthoMax = orthoProjection(pmax, p);

			ROS_DEBUG_NAMED("segmentsConstrutcion", "Pmin (%f, %f), Pmax (%f, %f)", pmin.getX(), pmin.getY(), pmax.getX(), pmax.getY());
			ROS_DEBUG_NAMED("segmentsConstrutcion", "Omin (%f, %f), Omax (%f, %f)", orthoMin.x, orthoMin.y, orthoMax.x, orthoMax.y);

			segm.setMinPoint(orthoMin);
			segm.setMaxPoint(orthoMax);

			ROS_DEBUG_NAMED("segmentsConstrutcion", "Taille du segment: %f", objLength);

			segm.setDistance(objLength);
			tabSegments.push_back(segm);

			// TODO: Remove below in release
			ROS_DEBUG_NAMED("segmentsConstrutcion", "DEBUG Segment for Object #%d", object_idx);
			std::stringstream rangesValues;
			for (int j = segm.getMinRanges(); j != segm.getMaxRanges(); j++)
			{
				rangesValues << " " << j << ": " << ranges[j] << " |";
			}
			ROS_DEBUG_NAMED("segmentsConstrutcion", "DEBUG %s", rangesValues.str().c_str());
		}

		rangesStart_idx += pointsVector_it->size();
		object_idx++;
	}

	ROS_DEBUG_NAMED("segmentsConstrutcion", "Nombre de segments d environ 70 cm trouvés: %zu", tabSegments.size());

	// Only the nearest segment is kept
	if (tabSegments.size() > 1)
	{
		Segment s;
		s = tabSegments[nearestSegment(tabSegments, ranges)];
		tabSegments.clear();
		tabSegments.push_back(s);
	}

	return tabSegments;
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

int FinalApproaching::nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges)
{
	int nearest = 0;
	std::vector<Segment>::iterator it;
	for (int i = 0; i < tabSegments.size(); i++)
	{
		// TODO: Check what are thoses errors messages ??
		// ROS_ERROR_COND(tabSegments.size() <= i || i < 0, "OOPS, Bad index vector acess, after line %d in file %s",
		// __LINE__, __FILE__);
		// ROS_ERROR_COND(ranges.size() != 513, "OOPS, Bad index vector acess, after line %d in file %s", __LINE__,
		// __FILE__);
		if (tabSegments[i].distanceLaserSegment(ranges) < tabSegments[nearest].distanceLaserSegment(ranges))
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

int FinalApproaching::correspondingId(std::vector<int> allPossibleId, std::vector<int> arTagId,
                                      std::vector<float> arTagDistance)
{
	int correspondingId = -1;
	std::vector<int> ids;
	ids.clear();
	if (!arTagId.empty())
	{
		;
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

int FinalApproaching::asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k,
                                           ArTagFA &arTag)
{
	int avancementArTag = 0;

	// float errY = 0.0;
	// float errX = 0.0;
	// float errTheta = 0.0;

	//    std::string tf_prefix;
	//    m_nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
	//    if (tf_prefix.size() != 0)
	//    {
	//        tf_prefix += "/";
	//    }

	//    tf::StampedTransform transform;
	//    try
	//    {
	//        g_tf_listener->lookupTransform(arTag.getFrame(), tf_prefix+"base_link", arTag.getStamp(), transform);
	//    }
	//    catch (tf::TransformException ex)
	//    {
	//        ROS_WARN("%s",ex.what());
	//        return;
	//    }

	//    double yaw = tf::getYaw(transform.getRotation());

	//        p.x     = px[k]*cos(yaw) - it.y*sin(yaw) + transform.getOrigin().x();
	//        p.y     = px[k]*sin(yaw) + it.y*cos(yaw) + transform.getOrigin().y();
	//        p.theta = it.theta + yaw;

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



int FinalApproaching::asservissementAngle(final_approach_msg::plotDataFA &plotData, ros::Publisher pubMvt, float angle){
	geometry_msgs::Twist msg;
	float err = angle - 0.01;
	plotData.angleErr = std::abs(err);
	if(std::abs(err) < m_laserYawPidThreshold())
	{
		msg.angular.z = 0;
		plotData.angleCmd = msg.angular.z;
		pubMvt.publish(msg);
		return 1;
	}
	else
	{
		msg.angular.z = m_laserYawPid.update(err);
		plotData.angleCmd = msg.angular.z;
		pubMvt.publish(msg);
		return 0;
	}
}


int FinalApproaching::asservissementPositionY(final_approach_msg::plotDataFA &plotData, ros::Publisher pubMvt
							, float goal, float moyPos, float yLeft, float yRight)
{
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	float err = moyPos + goal;

	// Determiner ce que sont les deux premiers if
	ROS_DEBUG("Asserv Laser Y");
#ifdef NOT_SO_USELESS
	if(yLeft >= 0 && yRight >= 0)
	{
		ROS_DEBUG("GO Full Right");
		msg.linear.y = 0.25;
		plotData.YErr = 2; // 2 >> error
        plotData.YCmd = msg.linear.y;
        pubMvt.publish(msg);
		return 0;
	}
	else if(yLeft <= 0 && yRight <= 0)
	{
		ROS_DEBUG("GO Full Left");
		msg.linear.y = -0.25;
		plotData.YErr = 2; // 2 >> error
		plotData.YCmd = msg.linear.y;
        pubMvt.publish(msg);
		return 0;
	}
	else
	{
#endif
		ROS_DEBUG("Asserv standard");
		plotData.YErr = std::abs(err);
		ROS_DEBUG("Asserv standard. Erreur: %f", std::abs(err));
		// TODO: Paramétrer seuil
		if(std::abs(err) < m_laserYPidThreshold())
		{
			msg.linear.y = 0;
			plotData.YCmd = msg.linear.y;
			pubMvt.publish(msg);
			return 1;
		}
		else
		{
			msg.linear.y = m_laserYPid.update(err);
			plotData.YCmd = msg.linear.y;
			pubMvt.publish(msg);
			return 0;
		}
#ifdef NOT_SO_USELESS
	}
#endif
}


int FinalApproaching::asservissementPositionX(final_approach_msg::plotDataFA &plotData,ros::Publisher pubMvt, float distance, float goal){
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.angular.z = 0;
	float err = distance - goal;
	plotData.XErr = std::abs(err);

	ROS_DEBUG("Asserv Laser X");
	if(std::abs(err) < m_laserXPidThreshold())
	{
		msg.linear.x = 0;
		plotData.XCmd = msg.linear.x;
		pubMvt.publish(msg);
		return 2;
	}
	else
	{
		if(std::abs(err) < 0.003)
		{
			msg.linear.x = m_laserXPid.update(err);
			plotData.XCmd = msg.linear.x;
			pubMvt.publish(msg);
			return 1;
		}
		else
		{
			msg.linear.x = m_laserXPid.update(err);
			plotData.XCmd = msg.linear.x;
			pubMvt.publish(msg);
			return 0;
		}
	}
}
