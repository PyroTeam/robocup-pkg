#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>


#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <manager_msg/finalApproachingAction.h>

#include <cmath>
#include <vector>
#include <list>

#include "SimFinalApproach.h"



SimFinalApproach::SimFinalApproach(std::string name):
	m_as(m_nh, name, boost::bind(&SimFinalApproach::executeCB, this, _1), false)
	, m_actionName(name)
{
	m_as.registerPreemptCallback(boost::bind(&SimFinalApproach::preemptCB, this));
	m_as.start();

	m_pubMvt = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
	m_landmarksSub = m_nh.subscribe("objectDetection/landmarks", 1, &SimFinalApproach::landmarksCallback, this);
}

SimFinalApproach::~SimFinalApproach(void){}

void SimFinalApproach::preemptCB()
{
	ROS_INFO("%s: Preempted", m_actionName.c_str());
	m_as.setPreempted();
}

void SimFinalApproach::landmarksCallback(const deplacement_msg::Machines& msg)
{
	/* XXX : Maybe this vector could be updated only once, because mps position should not change with mocked detecion */

  for (auto &it : msg.landmarks)
  {
    	m_mps.push_back(it.pose);
	}
}

void SimFinalApproach::executeCB(const manager_msg::finalApproachingGoalConstPtr &goal)
{
/* Environment */
	ros::Rate loopRate(30);
	bool succeeded = false;

	geometry_msgs::Pose2D mpsPose;
	geometry_msgs::Pose2D robotPose;
	geometry_msgs::Pose2D targetPose;
	geometry_msgs::Twist movement;

/* Init */
	m_feedback.percent_complete = 0;

/* Algo */
	while (ros::ok() && m_as.isActive() && !succeeded)
	{

		/* Get pose (measure) */


		/* Get goal (setpoint) */
		m_type = goal->type;
		m_side = goal->side;
		m_parameter = goal->parameter;

		/* Get error */

		/* Get command */

		/* Publish command */



		/* Success */
		succeeded = true;



		/* Publish feedback */
		m_as.publishFeedback(m_feedback);

		/* Spin and sleep */
		ros::spinOnce();
		loopRate.sleep();
	}

	ROS_INFO("%s: Finished", m_actionName.c_str());
	/* If not preempted */
	if (m_as.isActive())
	{
		/* Did we finished the final approach ? */
		if (succeeded)
		{
			m_result.success = true;
			m_result.state = 0;
			ROS_INFO("%s: Succeeded", m_actionName.c_str());
			m_as.setSucceeded(m_result);
		}
		else
		{
			m_result.success = false;
			ROS_WARN("%s: Aborted",m_actionName.c_str());
			m_as.setAborted(m_result);
		}
	}
	else
	{
		ROS_WARN("%s: Preempted",m_actionName.c_str());
	}


// Disabled code
#if (0 == 1)
	ros::Rate loopRate(100);
	approche_finale_msg::plotDataFA plotData;
	//general initialisation
	bool success = true;
	feedback.percent_complete = 0;
	geometry_msgs::Twist msgTwist;
	bool usefulInfo = false;
	//initialisation for bumper
	BumperListener bp;
	//initialisation for odometry
	OdomFA odom;
	float initOrientation = odom.getOrientationZ();
	bool initOdom = false;
	//initialisation for ARTag
	ArTagFA at;
	int k=-1;
	int phase = 0;
	int avancementArTag = 0;
	std::vector<int> id;
	std::vector<float> px = at.getPositionX();
	std::vector<float> pz = at.getPositionZ();
	std::vector<float> oz = at.getOrientationZ();
	std::vector<float> arTagDistance = at.getDistance();
	//initialisation for sharps
	Sharps sharps;
	bool obstacle = false;
	std::vector<bool> allObstacles = sharps.getObstacle();
	//initialisation for team and game phase
	GameStateFA gameState;
	int teamColor;
	nh.param<int>("teamColor",teamColor,0);
	//initialisation for laserscan
	laserScan ls;
	int a=0, b=0, c=0, cpt=0;
	float positionY=0, gradient=0, ortho=0, moyY=0, moyO=0;
    int j = 0;
    std::list<float> listPositionY, listOrtho;
    //to get goal action
	ROS_INFO("%s: Executing, creating SimFinalApproach sequence of type %i with side %i and parameter %i", actionName.c_str(), goal->type, goal->side,goal->parameter);
	m_type = goal->type;
	m_side = goal->side;
	m_parameter = goal->parameter;
	//at least one callback for odometry and gamestate
	while(ros::ok() && !bp.getState() && !usefulInfo)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}
		if(odom.getTurn() /*&& gameState.getAlready()*/)
		{
			usefulInfo = true;
		}
	}
	//loop to find a correct ARTag
	std::vector<int> allPossibleId = idWanted(teamColor,0/*gameState.getPhase()*/);
	while(ros::ok() && !bp.getState() && k==-1 && phase!=3)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}
		if(at.getFoundId())
		{
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
		}
		if(phase == 0)
		{
			initOrientation = odom.getOrientationZ();
			phase++;
		}
		msgTwist.angular.z = cameraScanVelocity(phase);
		float newOrientation = odom.getOrientationZ()-initOrientation;
		phase = phaseDependingOnOrientation(newOrientation,phase);
		m_pubMvt.publish(msgTwist);
	}
	//lock loop with ARTag camera
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==0 && obstacle==false)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}
		//if at least one ARTag found
		if(at.getFoundId())
		{
			px=at.getPositionX();
			pz=at.getPositionZ();
			oz=at.getOrientationZ();
			k=correspondingId(allPossibleId,at.getId(),at.getDistance());
			ROS_DEBUG("taille de px: %d de pz: %d de oz: %d et valeur de k: %d",
			          (int)px.size(),(int)pz.size(),(int)oz.size(),k);
			avancementArTag=finalApproaching::asservissementCamera(px,pz,oz,k);
		}
		allObstacles = sharps.getObstacle();
		obstacle = false;//obstacleDetection(allObstacles, k, oz, pz);
	}
	ROS_INFO("Waiting a complete laserscan");
	//lock loop with laserscan
	while(ros::ok() && !bp.getState() && phase!=3 && avancementArTag==1 && c!=2 && obstacle==false)
	{
		// make sure that the action hasn't been canceled
		if (!as.isActive())
		{
      		return;
  		}
		// if the scan is complete
		if(ls.getRanges().size() == 513)
		{
			std::vector<float> ranges = ls.getRanges();
			float angleMin = ls.getAngleMin();
			double angleInc = ls.getAngleInc();
			float rangeMin = ls.getRangeMin();
			float rangeMax = ls.getRangeMax();
			std::list<std::vector<Point> > tabPoints = objectsConstruction(ranges, angleMin, angleInc, rangeMin, rangeMax);
			std::vector<Segment> tabSegments = segmentsConstruction(tabPoints, ranges, angleMin, angleInc);
			//at least one segment found
			if(tabSegments.size()>0)
			{
				Segment seg = tabSegments[0];
				ROS_DEBUG("min seg: %d max seg %d",seg.getMinRanges(),seg.getMaxRanges());
				//if no error about the laserscan data
				if(seg.getMinRanges()>=0 && seg.getMinRanges()<=513 && seg.getMaxRanges()>=0 && seg.getMaxRanges()<=513)
				{
					gradient = seg.getGradient();
					ROS_DEBUG("gradient du segment le plus proche : %f",gradient);
					positionY = positionYLaser(seg, ranges, angleMin, angleInc);
					ROS_INFO("l objet se trouve a environ %f m du bord",positionY);
					ortho = distanceOrtho(seg, ranges, angleMin, angleInc);
					ROS_DEBUG("distance orthogonale laser-machine : %f",ortho);

					//to do an average
					if(j<1)
					{
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						j++;
					}
					else
					{
						listPositionY.pop_front();
						listOrtho.pop_front();
						listPositionY.push_back(positionY);
						listOrtho.push_back(ortho);
						moyY = moy(listPositionY);
						moyO = moy(listOrtho);
						//to be in front of the machine
						a = asservissementAngle(plotData,m_pubMvt,gradient);
						int min = seg.getMinRanges();
						int max = seg.getMaxRanges();
						Point gauche(ranges[max],angleMin+(double)min*angleInc);
						Point droite(ranges[min],angleMin+(double)max*angleInc);
						plotData.XLeft = gauche.getX();
						plotData.YLeft = gauche.getY();
						plotData.XRight = droite.getX();
						plotData.YRight = droite.getY();
						if((a == 1) && (cpt < 200))
						{
							//to move on the Y axis of the robot
							b = asservissementPositionY(plotData,m_pubMvt,moyY,objectifY(),gauche.getY(),droite.getY());
							ROS_INFO("positionY: %f",positionY);
							//listPositionY.clear();
							if(b==1)
							{
								cpt++;
							}
						}
						if(a == 1 && cpt >= 200)
						{
							//to move on the X axis of the robot
							c = asservissementPositionX(plotData,m_pubMvt,ortho,objectifX());
						}
					}
				}
				m_plot.publish(plotData);
				feedback.percent_complete = avancement(a,b,c);
				as.publishFeedback(feedback);
				ros::spinOnce();
				ROS_DEBUG("etat du bumper: %d ",bp.getState());
				loopRate.sleep();
			}
		}
	}
	//reinitialisation
	geometry_msgs::Twist stop;
	a=0;
	b=0;
	c=0;
	cpt=0;
	j=0;
	avancementArTag=0;
	phase=0;
	listPositionY.clear();
	listOrtho.clear();
	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.angular.z = 0;
	m_pubMvt.publish(stop);
	//if any problem
	if(bp.getState() || phase==3 || obstacle==true)
	{
		if(bp.getState())
		{
			ROS_WARN("OBSTACLE RENCONTRE\n");
			result.state = 1;
		}
		if(phase == 3)
		{
			ROS_WARN("UN BALAYAGE COMPLET SANS ARTAG CORRECT\n");
			result.state = 3;
		}
		if(obstacle==true)
		{
			ROS_WARN("OBSTACLE TROP PROCHE\n");
			result.state = 2;
		}
		success=false;
	}
	if(success)
	{
		result.success = true;
		result.state = 0;
		ROS_INFO("%s: Succeeded", actionName.c_str());
		as.setSucceeded(result);
	}
	else
	{
		result.success = false;
		ROS_INFO("%s: Aborted",actionName.c_str());
		as.setAborted(result);
	}
#endif
}
