/**
 * \file 			FinalApproaching.h
 * \class			FinalApproaching
 * \brief			classe principale de l approche finale
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef FINALAPPROACHING_H
#define FINALAPPROACHING_H

#include "Point.h"
#include "Segment.h"
#include "ArTagFA.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include <manager_msg/FinalApproachingAction.h>
#include <approche_finale_msg/plotDataFA.h>
#include <manager_msg/FinalApproachingAction.h>

#include <vector>


enum team_e
{
	CYAN = 0,
	MAGENTA = 1
};

enum mpsARTags_e
{
	C_CS1_IN	=	1,
	C_CS1_OUT	=	2,
	C_CS2_IN	=	17,
	C_CS2_OUT	=	18,
	C_RS1_IN	=	33,
	C_RS1_OUT	=	34,
	C_RS2_IN	=	177,
	C_RS2_OUT	=	178,
	C_BS_IN		=	65,
	C_BS_OUT	=	66,
	C_DS_IN		=	81,
	C_DS_OUT	=	82,

	M_CS1_IN	=	97,
	M_CS1_OUT	=	98,
	M_CS2_IN	=	113,
	M_CS2_OUT	=	114,
	M_RS1_IN	=	129,
	M_RS1_OUT	=	130,
	M_RS2_IN	=	145,
	M_RS2_OUT	=	146,
	M_BS_IN		=	161,
	M_BS_OUT	=	162,
	M_DS_IN		=	49,
	M_DS_OUT	=	50
};

// Unique identifier for markers
enum markerId_e
{
	SEGMENT_MARKER_ID,
	ARTAG_MARKER_ID
};


class FinalApproaching
{
	protected:
		ros::NodeHandle m_nh;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<manager_msg::FinalApproachingAction> m_as;
		std::string m_actionName;
		// Create messages that are used to published m_feedback/m_result
		manager_msg::FinalApproachingFeedback m_feedback;
		manager_msg::FinalApproachingResult m_result;
		ros::Publisher m_pubMvt;
		ros::Publisher m_markerPub;
		ros::Publisher m_plot;
		int m_type;
		int m_side;
		int m_parameter;


	public:
		FinalApproaching(std::string name):
		m_as(m_nh, name, boost::bind(&FinalApproaching::executeCB, this, _1), false), m_actionName(name)
		{

			m_pubMvt = m_nh.advertise<geometry_msgs::Twist>("hardware/cmd_vel", 1);
			m_plot = m_nh.advertise<approche_finale_msg::plotDataFA>("rosplot/plotDataFA", 1000);
			m_markerPub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

			m_as.registerPreemptCallback(boost::bind(&FinalApproaching::preemptCB, this));
			m_as.start();
		}

		~FinalApproaching(void);

		/**
		 * \brief      Vérifie si l'action a été annulée
		 */
		void preemptCB();

		void executeCB(const manager_msg::FinalApproachingGoalConstPtr &goal);

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
		 *             lasers
		 *
		 * \param[in]  ranges    The ranges
		 * \param[in]  angleMin  The angle minimum
		 * \param[in]  angleInc  The angle increment
		 * \param[in]  rangeMin  The range minimum
		 * \param[in]  rangeMax  The range maximum
		 *
		 * \return     { description_of_the_return_value }
		 */
		std::list<std::vector<Point> > objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc, float rangeMin, float rangeMax);

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
		std::vector<Segment> segmentsConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc);

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
		float objectLength(int i, int j, std::list<std::vector<Point> > tabPoints, std::vector<float> ranges
							, float angleMin, double angleInc);

		/**
		 * \brief      Détermine le segment le plus proche du laser
		 *
		 * \param[in]  tabSegments  The tab segments
		 * \param[in]  ranges       The ranges
		 *
		 * \return     le numero du  segment le plus proche du laser
		 */
		int nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges);

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
		 * \param[in]  team   The team
		 * \param[in]  phase  The phase
		 *
		 * \return     un tableau d'entiers conteant tous les ids possibles à
		 *             rechercher
		 */
		std::vector<int> idWanted(int team, int phase);

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
		 * \return     retourne 0 si c'est fini, 0 sinon
		 */
		int asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k, ArTagFA &arTag);

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
		bool obstacleDetection(std::vector<bool> allObstacles, int k,std::vector<float> oz, std::vector<float> pz);

};


#endif
