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
		ros::Publisher m_plot;
		int m_type;
		int m_side;
		int m_parameter;
		

	public:
		finalApproaching(std::string name) :
		as(nh, name, boost::bind(&finalApproaching::executeCB, this, _1), false),actionName(name)
		{
			as.registerPreemptCallback(boost::bind(&finalApproaching::preemptCB, this));
			as.start();
		}

		~finalApproaching(void);

/**
 *	\brief		Vérifie si l'action a été annulée
 */
		void preemptCB();
		
		void executeCB(const manager_msg::finalApproachingGoalConstPtr &goal);
		
/**
 *  \brief		Avancement de l action
 *  \return		un entier représentant l'évolution de l'action
 */		
		int avancement(int a, int b, int c);
		
/**
 *  \brief		Détermine la consigne en x (repère robot)
 *  \return		consigne en x
 */
		float objectifX();
		
/**
 *  \brief		Détermine la consigne en y (repère robot)
 *  \return		consigne en y
 */
		float objectifY();

/**
 *  \brief		Crée les objets a partir du tableau de points des données lasers
 */		
		std::list<std::vector<Point> > objectsConstruction(std::vector<float> ranges, float angleMin, double angleInc, float rangeMin, float rangeMax);

/**
 *  \brief		Construit les différents segments vus par le laser
 */		
		std::vector<Segment> segmentsConstruction(std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc);

/**
 *  \brief		Calcule la longueur d un objet
 *  \return		la longueur de l objet
 */		
		float objectLength(int i, int j, std::list<std::vector<Point> > tabPoints, std::vector<float> ranges, float angleMin, double angleInc);

/**
 *  \brief		Détermine le segment le plus proche du laser
 *  \return		le numero du  segment le plus proche du laser
 */		
		int nearestSegment(std::vector<Segment> tabSegments, std::vector<float> ranges);

/**
 *  \brief		Détermine la distance orthogonale entre le laser et le segment
 *  \return		la distance orthogonale entre le laser et le segment
 */		
		float distanceOrtho(Segment s,std::vector<float> ranges,float angleMin, double angleInc);

/**
 *  \brief		position en y (repère robot) par rapport à l'extrémité à droite du segment
 *  \return		distance entre le bord droit de la machine et le projete orthogonal du laser sur la machine
 */		
		float positionYLaser(Segment s,std::vector<float> ranges, float angleMin, double angleInc);

/**
 *  \brief		détermine les artags à chercher suivants l'état du jeu
 *  \return		un tableau d'entiers conteant tous les ids possibles à rechercher
 */
		std::vector<int> idWanted(int team,int phase);

/**
 *  \brief		vérifie si l'artag vu est l'un des artags recherchés
 *  \return		l'indice du tableau des ids si cela correspond à un id recherché, -1 sinon
 */
		int correspondingId(std::vector<int> allPossibleId,std::vector<int> arTagId,std::vector<float> arTagDistance);

/**
 *  \brief		place le robot à 50cm (par rapport à la caméra) en face de l'artag
 *  \return		retourne 0 si c'est fini, 0 sinon
 */
		int asservissementCamera(std::vector<float> px, std::vector<float> pz, std::vector<float> oz, int k);

/**
 *  \brief		détermine la vitesse angulaire du robot pour effectuer un balayage
 *  \return		la vitesse angulaire en z (repère robot)
 */		
		float cameraScanVelocity(int phase);
		
/**
 *  \brief		détermine si on change de phase de balayage
 *  \return		la nouvelle phase de balayage
 */		
		int phaseDependingOnOrientation(float newOrientation, int phase);
		
/**
 *  \brief		détecte s'il y a un obstacle proche qui ne sont pas les roues des machines
 *  \return		retourne true si présence d'obstacle, sinon false
 */		
		bool obstacleDetection(std::vector<bool> allObstacles,int k,std::vector<float> oz,std::vector<float> pz);

};


#endif
