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
	ARTAG_MARKER_ID
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

	int m_type;
	int m_side;
	int m_parameter;
	enum team_e m_teamColor;

	// Laser X Asserv PID Parameters
	Parameter m_laserXPidKp;
	Parameter m_laserXPidKi;
	Parameter m_laserXPidKd;
	Parameter m_laserXPidThreshold;

	// Laser Y Asserv PID Parameters
	Parameter m_laserYPidKp;
	Parameter m_laserYPidKi;
	Parameter m_laserYPidKd;
	Parameter m_laserYPidThreshold;

	// Laser Yaw Asserv PID Parameters
	Parameter m_laserYawPidKp;
	Parameter m_laserYawPidKi;
	Parameter m_laserYawPidKd;
	Parameter m_laserYawPidThreshold;

	// PID
	common_utils::Pid m_laserXPid;
	common_utils::Pid m_laserYPid;
	common_utils::Pid m_laserYawPid;

	// Some useful infos
	Parameter m_mpsWidth;

  public:
	FinalApproaching(std::string name)
		: m_as(m_nh, name, boost::bind(&FinalApproaching::executeCB, this, _1), false), m_actionName(name)
		, m_laserXPidKp(m_nh, "navigation/FinalApproach/laserAsserv/xPid/Kp", 0.25)
		, m_laserXPidKi(m_nh, "navigation/FinalApproach/laserAsserv/xPid/Ki", 0)
		, m_laserXPidKd(m_nh, "navigation/FinalApproach/laserAsserv/xPid/Kd", 0)
		, m_laserXPidThreshold(m_nh, "navigation/FinalApproach/laserAsserv/xPid/threshold", 0.01)

		, m_laserYPidKp(m_nh, "navigation/FinalApproach/laserAsserv/yPid/Kp", 0.075)
		, m_laserYPidKi(m_nh, "navigation/FinalApproach/laserAsserv/yPid/Ki", 0)
		, m_laserYPidKd(m_nh, "navigation/FinalApproach/laserAsserv/yPid/Kd", 0)
		, m_laserYPidThreshold(m_nh, "navigation/FinalApproach/laserAsserv/yPid/threshold", 0.003)

		, m_laserYawPidKp(m_nh, "navigation/FinalApproach/laserAsserv/yawPid/Kp", 0.4)
		, m_laserYawPidKi(m_nh, "navigation/FinalApproach/laserAsserv/yawPid/Ki", 0)
		, m_laserYawPidKd(m_nh, "navigation/FinalApproach/laserAsserv/yawPid/Kd", 0)
		, m_laserYawPidThreshold(m_nh, "navigation/FinalApproach/laserAsserv/yawPid/threshold", 0.003)

		, m_laserXPid(m_laserXPidKp(), m_laserXPidKi(), m_laserXPidKd(), 1.0/g_loopFreq)
		, m_laserYPid(m_laserYPidKp(), m_laserYPidKi(), m_laserYPidKd(), 1.0/g_loopFreq)
		, m_laserYawPid(m_laserYawPidKp(), m_laserYawPidKi(), m_laserYawPidKd(), 1.0/g_loopFreq)

		, m_mpsWidth(m_nh, "navigation/FinalApproach/mps/width", 0.700)
	{
		refreshParams();

		// Advertise publishers
		m_pubMvt = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
		m_plot = m_nh.advertise<final_approach_msg::plotDataFA>("rosplot/plotDataFA", 1000);
		m_markerPub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

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
	 * \brief      Avancement de l action
	 *
	 * \param[in]  a     { parameter_description }
	 * \param[in]  b     { parameter_description }
	 * \param[in]  c     { parameter_description }
	 *
	 * \return     un entier représentant l'évolution de l'action
	 */
	int avancement(int a, int b, int c);

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
	 * \brief      Calcule la longueur d'un objet
	 *
	 * \param[in]  i          { parameter_description }
	 * \param[in]  j          { parameter_description }
	 * \param[in]  tabPoints  The tab points
	 * \param[in]  ranges     The ranges
	 * \param[in]  angleMin   The angle minimum
	 * \param[in]  angleInc   The angle increment
	 *
	 * \return     la longueur de l objet
	 */
	float objectLength(int i, int j, std::list<std::vector<Point> > tabPoints, std::vector<float> ranges,
	                   float angleMin, double angleInc);

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
	 * \param[in]  arTagId        The archive tag identifier
	 * \param[in]  arTagDistance  The archive tag distance
	 *
	 * \return     l'indice du tableau des ids si cela correspond à un id
	 *             recherché, -1 sinon
	 */
	int correspondingId(std::vector<int> allPossibleId, std::vector<int> arTagId, std::vector<float> arTagDistance);

	/**
	 * \brief      place le robot à 50cm (par rapport à la caméra) en face
	 *             de l'artag
	 *
	 * \param[in]  px     { parameter_description }
	 * \param[in]  pz     { parameter_description }
	 * \param[in]  oz     { parameter_description }
	 * \param[in]  k      { parameter_description }
	 * \param      arTag  The archive tag
	 *
	 * \return     retourne 1 si c'est fini, 0 sinon
	 */
	int asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k,
	                         ArTagFA &arTag);


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
};


#endif // _FINAL_APPROACH__FINALAPPROACHING__H_
