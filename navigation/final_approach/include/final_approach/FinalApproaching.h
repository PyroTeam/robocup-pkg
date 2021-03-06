/**
 * \file
 * \class      FinalApproaching
 * \brief      Classe principale de l'approche finale
 * \author     Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \author     Vergez Valentin (valentin.vergez@stormshield.eu)
 * \date       Create : 2015-04-20
 * \date       Last modified : 2016-05-19
 * \copyright  2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _FINAL_APPROACH__FINALAPPROACHING__H_
#define _FINAL_APPROACH__FINALAPPROACHING__H_

#include "final_approach/Point.h"
#include "final_approach/Segment.h"
#include "final_approach/ArTagFA.h"
#include "final_approach/LaserScan.h"
#include "final_approach/OdomFA.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <final_approach_msg/FinalApproachingAction.h>
#include <final_approach_msg/plotDataFA.h>
#include <common_utils/Parameter.h>
#include <common_utils/controller/Pid.h>

#include <vector>
#include <cmath>

/**
 * Fréquence des boucles en Hz
 */
const int g_loopFreq = 100;

/**
 * \brief      Team color
 */
enum team_e
{
	CYAN = 0,
	MAGENTA = 1
};

/**
 * \brief      MPS ARTag identifiers
 */
enum mpsARTags_e
{
	C_CS1_IN = 1,
	C_CS1_OUT = 2,
	C_CS2_IN = 17,
	C_CS2_OUT = 18,
	C_RS1_IN = 33,
	C_RS1_OUT = 34,
	C_RS2_IN = 177,
	C_RS2_OUT = 178,
	C_BS_IN = 65,
	C_BS_OUT = 66,
	C_DS_IN = 81,
	C_DS_OUT = 82,

	M_CS1_IN = 97,
	M_CS1_OUT = 98,
	M_CS2_IN = 113,
	M_CS2_OUT = 114,
	M_RS1_IN = 129,
	M_RS1_OUT = 130,
	M_RS2_IN = 145,
	M_RS2_OUT = 146,
	M_BS_IN = 161,
	M_BS_OUT = 162,
	M_DS_IN = 49,
	M_DS_OUT = 50
};

/**
 * \brief      Unique identifier for markers
 */
enum markerId_e
{
	SEGMENT_MARKER_ID,
	ARTAG_MARKER_ID,
	MID_POINT_ID
};

/**
 * \brief      Classe principale de l'approche finale
 *
 * \details    Cette classe gère l'action de l'approche finale ainsi que
 *             l'algorithme de l'approche finale.
 */
class FinalApproaching
{
  protected:
	ros::NodeHandle m_nh;

	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<final_approach_msg::FinalApproachingAction> m_as;
	std::string m_actionName;

	// Messages
	final_approach_msg::FinalApproachingFeedback m_feedback;
	final_approach_msg::FinalApproachingResult m_result;
	final_approach_msg::plotDataFA m_plotData;
	visualization_msgs::Marker m_marker;
	geometry_msgs::Twist m_msgTwist;

	// Publisher
	ros::Publisher m_pubMvt;
	ros::Publisher m_markerPub;
	ros::Publisher m_plot;

	// Subscriber
	ros::Subscriber m_control;

	int m_type;
	int m_side;
	int m_parameter;
	enum team_e m_teamColor;
	float m_controlTopicDist;
	float m_controlParamDist;
	bool m_Controlled;
	uint8_t m_mode;

	// Laser X Asserv PID Parameters
	Parameter m_laserXPidKp;
	Parameter m_laserXPidKi;
	Parameter m_laserXPidKd;
	Parameter m_laserXPidThreshold;
	Parameter m_laserXPidNbSuccessNeeded;

	// Laser Y Asserv PID Parameters
	Parameter m_laserYPidKp;
	Parameter m_laserYPidKi;
	Parameter m_laserYPidKd;
	Parameter m_laserYPidThreshold;
	Parameter m_laserYPidNbSuccessNeeded;

	// Laser Yaw Asserv PID Parameters
	Parameter m_laserYawPidKp;
	Parameter m_laserYawPidKi;
	Parameter m_laserYawPidKd;
	Parameter m_laserYawPidThreshold;
	Parameter m_laserYawPidNbSuccessNeeded;

	// Cam X Asserv PID Parameters
	Parameter m_camXPidKp;
	Parameter m_camXPidKi;
	Parameter m_camXPidKd;
	Parameter m_camXPidThreshold;
	Parameter m_camXPidNbSuccessNeeded;

	// Cam Y Asserv PID Parameters
	Parameter m_camYPidKp;
	Parameter m_camYPidKi;
	Parameter m_camYPidKd;
	Parameter m_camYPidThreshold;
	Parameter m_camYPidNbSuccessNeeded;

	// Cam Yaw Asserv PID Parameters
	Parameter m_camYawPidKp;
	Parameter m_camYawPidKi;
	Parameter m_camYawPidKd;
	Parameter m_camYawPidThreshold;
	Parameter m_camYawPidNbSuccessNeeded;

	// PID
	common_utils::Pid m_laserXPid;
	common_utils::Pid m_laserYPid;
	common_utils::Pid m_laserYawPid;

	// Some useful infos
	Parameter m_mpsWidth;

	// Les positions sur l'axe y sont renseignée côté INPUT de la machine
	Parameter m_yPoseS1;
	Parameter m_yPoseS2;
	Parameter m_yPoseS3;
	Parameter m_yPoseLANE_RS;
	Parameter m_yPoseLIGHT;
	Parameter m_yPoseLIGHT_OLD;
	Parameter m_yPoseCONVEYOR;

	Parameter m_xPoseLIGHT_OLD;
	Parameter m_xPoseCONVEYOR;

	Parameter m_skipAsservCamera;
	Parameter m_skipAsservLaser;

  public:
	FinalApproaching(std::string name)
		: m_as(m_nh, name, boost::bind(&FinalApproaching::executeCB, this, _1), false), m_actionName(name)

		, m_laserXPidKp(m_nh, "navigation/finalApproach/laserAsserv/xPid/Kp", 0.25)
		, m_laserXPidKi(m_nh, "navigation/finalApproach/laserAsserv/xPid/Ki", 0)
		, m_laserXPidKd(m_nh, "navigation/finalApproach/laserAsserv/xPid/Kd", 0)
		, m_laserXPidThreshold(m_nh, "navigation/finalApproach/laserAsserv/xPid/threshold", 0.01)
		, m_laserXPidNbSuccessNeeded(m_nh, "navigation/finalApproach/laserAsserv/xPid/nbSuccessNeeded", 3.0)

		, m_laserYPidKp(m_nh, "navigation/finalApproach/laserAsserv/yPid/Kp", 0.075)
		, m_laserYPidKi(m_nh, "navigation/finalApproach/laserAsserv/yPid/Ki", 0)
		, m_laserYPidKd(m_nh, "navigation/finalApproach/laserAsserv/yPid/Kd", 0)
		, m_laserYPidThreshold(m_nh, "navigation/finalApproach/laserAsserv/yPid/threshold", 0.003)
		, m_laserYPidNbSuccessNeeded(m_nh, "navigation/finalApproach/laserAsserv/yPid/nbSuccessNeeded", 3.0)

		, m_laserYawPidKp(m_nh, "navigation/finalApproach/laserAsserv/yawPid/Kp", 0.4)
		, m_laserYawPidKi(m_nh, "navigation/finalApproach/laserAsserv/yawPid/Ki", 0)
		, m_laserYawPidKd(m_nh, "navigation/finalApproach/laserAsserv/yawPid/Kd", 0)
		, m_laserYawPidThreshold(m_nh, "navigation/finalApproach/laserAsserv/yawPid/threshold", 0.003)
		, m_laserYawPidNbSuccessNeeded(m_nh, "navigation/finalApproach/laserAsserv/yawPid/nbSuccessNeeded", 3.0)

		, m_camXPidKp(m_nh, "navigation/finalApproach/camAsserv/xPid/Kp", 0.25)
		, m_camXPidKi(m_nh, "navigation/finalApproach/camAsserv/xPid/Ki", 0)
		, m_camXPidKd(m_nh, "navigation/finalApproach/camAsserv/xPid/Kd", 0)
		, m_camXPidThreshold(m_nh, "navigation/finalApproach/camAsserv/xPid/threshold", 0.01)
		, m_camXPidNbSuccessNeeded(m_nh, "navigation/finalApproach/camAsserv/xPid/nbSuccessNeeded", 3.0)

		, m_camYPidKp(m_nh, "navigation/finalApproach/camAsserv/yPid/Kp", 0.075)
		, m_camYPidKi(m_nh, "navigation/finalApproach/camAsserv/yPid/Ki", 0)
		, m_camYPidKd(m_nh, "navigation/finalApproach/camAsserv/yPid/Kd", 0)
		, m_camYPidThreshold(m_nh, "navigation/finalApproach/camAsserv/yPid/threshold", 0.003)
		, m_camYPidNbSuccessNeeded(m_nh, "navigation/finalApproach/camAsserv/yPid/nbSuccessNeeded", 3.0)

		, m_camYawPidKp(m_nh, "navigation/finalApproach/camAsserv/yawPid/Kp", 0.4)
		, m_camYawPidKi(m_nh, "navigation/finalApproach/camAsserv/yawPid/Ki", 0)
		, m_camYawPidKd(m_nh, "navigation/finalApproach/camAsserv/yawPid/Kd", 0)
		, m_camYawPidThreshold(m_nh, "navigation/finalApproach/camAsserv/yawPid/threshold", 0.003)
		, m_camYawPidNbSuccessNeeded(m_nh, "navigation/finalApproach/camAsserv/yawPid/nbSuccessNeeded", 3.0)

		, m_laserXPid(m_laserXPidKp(), m_laserXPidKi(), m_laserXPidKd(), 1.0/g_loopFreq)
		, m_laserYPid(m_laserYPidKp(), m_laserYPidKi(), m_laserYPidKd(), 1.0/g_loopFreq)
		, m_laserYawPid(m_laserYawPidKp(), m_laserYawPidKi(), m_laserYawPidKd(), 1.0/g_loopFreq)

		, m_mpsWidth(m_nh, "navigation/finalApproach/mps/width", 0.700)

		, m_yPoseS1(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/S1", -0.07)
		, m_yPoseS2(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/S2", -0.175)
		, m_yPoseS3(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/S3", -0.27)
		, m_yPoseLANE_RS(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/LANE_RS", -0.26)
		, m_yPoseLIGHT(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/LIGHT", 0.0)
		, m_yPoseLIGHT_OLD(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/LIGHT_OLD", 0.0)
		, m_yPoseCONVEYOR(m_nh, "navigation/finalApproach/mps/targetPoses/yAxis/CONVEYOR", 0.0225)

		, m_xPoseLIGHT_OLD(m_nh, "navigation/finalApproach/mps/targetPoses/xAxis/LIGHT_OLD", -0.36)
		, m_xPoseCONVEYOR(m_nh, "navigation/finalApproach/mps/targetPoses/xAxis/CONVEYOR", -0.16)

		, m_skipAsservCamera(m_nh, "navigation/finalApproach/skipAsservCamera", 0.0)
		, m_skipAsservLaser(m_nh, "navigation/finalApproach/skipAsservLaser", 0.0)

		, m_controlTopicDist(std::nanf(""))
		, m_Controlled(false)
	{
		refreshParams();

		// Advertise publishers
		m_pubMvt = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
		m_plot = m_nh.advertise<final_approach_msg::plotDataFA>("rosplot/plotDataFA", 1000);
		m_markerPub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

		// Subscribe
		m_control = m_nh.subscribe("navigation/finalApproachControl", 1, &FinalApproaching::controlCallback, this);

		// Start ActionServer
		m_as.registerPreemptCallback(boost::bind(&FinalApproaching::preemptCB, this));
		m_as.start();
	}

	~FinalApproaching(void);

	/**
	 * \brief      Vérifie si l'action a été annulée
	 */
	void preemptCB();

	void executeCB(const final_approach_msg::FinalApproachingGoalConstPtr &goal);


	/**
	 * \brief      Détermine la consigne en x (repère robot)
	 *
	 * \return     consigne en
	 */
	float objectifX();

	/**
	 * \brief      Détermine la consigne en y (repère robot)
	 *
	 * \return     consigne en
	 */
	float objectifY();

	/**
	 * \brief      Crée les objets a partir du tableau de points des données
	 *             lasers.
	 *
	 * \detail La segmentation s'effectue sur la différence de range entre deux points
	 * voisins, si elle est supérieur à margin (5 centimètres recommandés), un nouvel
	 * objet est créé. Les points comportant un range hors des limites de rangeMin /
	 * rangeMax seront ignorés (mais pas leurs voisins, pour l'instant)
	 *
	 * \param[in]  ranges    The ranges
	 * \param[in]  angleMin  The angle minimum
	 * \param[in]  angleInc  The angle increment
	 * \param[in]  rangeMin  The range minimum
	 * \param[in]  rangeMax  The range maximum
	 * \param[in]  margin    The margin
	 *
	 * \return     { description_of_the_return_value }
	 */
	std::list<std::vector<Point> > objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc,
	                                                   float rangeMin, float rangeMax, float margin = 0.10);

	/**
	 * \brief      Construit les différents segments vus par le laser
	 *
	 * \param[in]  tabPoints  The tab points
	 * \param[in]  ranges     The ranges
	 * \param[in]  angleMin   The angle minimum
	 * \param[in]  angleInc   The angle increment
	 *
	 * \return     { description_of_the_return_value }
	 */
	Segment segmentConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges,
	                                          float angleMin, double angleInc);

	void refreshParams(void)
	{
		std::string teamColor_str;
		if (!m_nh.getParamCached("teamColor", teamColor_str))
		{
			teamColor_str = "cyan";
			ROS_WARN("Unable to get teamColor parameter. Will use default: %s", teamColor_str.c_str());
		}

		m_teamColor = (teamColor_str == "magenta") ? MAGENTA : CYAN;

		m_laserXPid.setKp(m_laserXPidKp());
		m_laserXPid.setKi(m_laserXPidKi());
		m_laserXPid.setKd(m_laserXPidKd());

		m_laserYPid.setKp(m_laserYPidKp());
		m_laserYPid.setKi(m_laserYPidKi());
		m_laserYPid.setKd(m_laserYPidKd());

		m_laserYawPid.setKp(m_laserYawPidKp());
		m_laserYawPid.setKi(m_laserYawPidKi());
		m_laserYawPid.setKd(m_laserYawPidKd());
	}

	/**
	 * \brief      Longueur entre les deux extrémités d'un "objet"
	 *
	 * \param      pointsVector  The points vector (ou objet)
	 *
	 * \return     Distance en mètres
	 */
	float objectLengthNew(std::vector<Point> &pointsVector);

	/**
	 * \brief      Détermine le segment le plus proche du laser
	 *
	 * \param[in]  tabSegments  The tab segments
	 * \param[in]  ranges       The ranges
	 *
	 * \return     le numero du  segment le plus proche du laser
	 */
	int nearestSegment(std::vector<Segment> tabSegments);

	/**
	 * \brief      Détermine la distance orthogonale entre le laser et le
	 *             segment
	 *
	 * \param[in]  s         { parameter_description }
	 * \param[in]  ranges    The ranges
	 * \param[in]  angleMin  The angle minimum
	 * \param[in]  angleInc  The angle increment
	 *
	 * \return     la distance orthogonale entre le laser et le segment
	 */
	float distanceOrtho(Segment s, std::vector<float> ranges, float angleMin, double angleInc);

	/**
	 * \brief      position en y (repère robot) par rapport à l'extrémité à
	 *             droite du segment
	 *
	 * \param[in]  s         { parameter_description }
	 * \param[in]  ranges    The ranges
	 * \param[in]  angleMin  The angle minimum
	 * \param[in]  angleInc  The angle increment
	 *
	 * \return     distance entre le bord droit de la machine et le projete
	 *             orthogonal du laser sur la machine
	 */
	float positionYLaser(Segment s, std::vector<float> ranges, float angleMin, double angleInc);

	/**
	 * \brief      détermine les artags à chercher suivants l'état du jeu
	 *
	 * \param[in]  phase  The phase
	 *
	 * \return     un tableau d'entiers conteant tous les ids possibles à
	 *             rechercher
	 */
	std::vector<int> idWanted(int phase);

	/**
	 * \brief      vérifie si l'artag vu est l'un des artags recherchés
	 *
	 * \param[in]  allPossibleId  The all possible identifier
	 * \param[in]  arTagId        The artag identifier
	 * \param[in]  arTagDistance  The artag distance
	 *
	 * \return     l'indice du tableau des ids si cela correspond à un id
	 *             recherché, -1 sinon
	 */
	int correspondingId(std::vector<int> allPossibleId, std::vector<int> arTagId, std::vector<float> arTagDistance);

	/**
	 * \brief      récupère l'index du vecteur d'arTags correspondant à l'arTag souhaité le plus
	 *             proche
	 *
	 * \param[in]  allPossibleId  The all possible identifier
	 * \param[in]  arTags         The artags vector
	 *
	 * \return     l'indice du tableau des ids si cela correspond à un id recherché, -1 sinon
	 */
	int correspondingId(std::vector<int> allPossibleId, std::vector<arTag_t> arTags);

	/**
	 * \brief      place le robot à 50cm (par rapport à la caméra) en face de
	 *             l'artag
	 *
	 * \param[in]  target  The targeted arTag
	 *
	 * \return     true if finished, false otherwise
	 */
	bool asservissementCameraNew(const arTag_t &target);

	/**
	 * \brief      détermine la vitesse angulaire du robot pour effectuer un
	 *             balayage
	 *
	 * \param[in]  phase  The phase
	 *
	 * \return     la vitesse angulaire en z (repère robot)
	 */
	float cameraScanVelocity(int phase);

	/**
	 * \brief      détermine si on change de phase de balayage
	 *
	 * \param[in]  newOrientation  The new orientation
	 * \param[in]  phase           The phase
	 *
	 * \return     la nouvelle phase de balayage
	 */
	int phaseDependingOnOrientation(float newOrientation, int phase);

	/**
	 * \brief      détecte s'il y a un obstacle proche qui ne sont pas les
	 *             roues des machines
	 *
	 * \param[in]  allObstacles  The all obstacles
	 * \param[in]  k             { parameter_description }
	 * \param[in]  oz            { parameter_description }
	 * \param[in]  pz            { parameter_description }
	 *
	 * \return     retourne true si présence d'obstacle, sinon false
	 */
	bool obstacleDetection(std::vector<bool> allObstacles, int k, std::vector<float> oz, std::vector<float> pz);

	/**
	 * \brief      Publie un message de vitesse nulle sur cmd_vel (pour stopper
	 *             le robot)
	 */
	void stopRobot(void);

	/**
	 *  \brief		permet d asservir en angle
	 *  \return		etat d avancement de l asservissement
	 */
	bool asservissementAngle(float setpoint, float measure);

	/**
	 *  \brief		permet d asservir en y (repere laser)
	 *  \return		etat d avancement de l asservissement
	 */
	bool asservissementPositionY(float setpoint, float measure);

	/**
	 *  \brief		permet d asservir en x (repere laser)
	 *  \return		etat d avancement de l asservissement
	 */
	bool asservissementPositionX(float setpoint, float measure);

	/**
	 * \brief      Publie un marker line_list (unique) représentant le segment
	 *
	 * \param      ls    Une référence sur laserScan pour récupérer quelques infos
	 * \param      seg   The segment
	 */
	void publishSegmentMarker(LaserScan &ls, Segment &seg);

	/**
	 * \brief      Affiche avec ROS_LOG le résultat de l'approche finale. Ne fonctionne qu'en input
	 *
	 * \param      odom      The odom
	 * \param[in]  mpsX      The mps x
	 * \param[in]  mpsY      The mps y
	 * \param[in]  mpsTheta  The mps theta
	 */
	void debugFinalApproachResult(OdomFA &odom, float mpsX = 0, float mpsY = 2.5, float mpsTheta = 0.0);

	/**
	 * \brief      Controlled Final Approach callback
	 *
	 * \param[in]  msg   The message
	 */
	void controlCallback(const std_msgs::Float32::ConstPtr& msg)
	{
		m_controlTopicDist = msg->data;
	}
};


#endif // _FINAL_APPROACH__FINALAPPROACHING__H_
