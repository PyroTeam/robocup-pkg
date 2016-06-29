/**
 * \file 			arTagFA.h
 * \class			ArTagFA
 * \brief			classe récupérant les données du topic /ar_pose_markers
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-05-25
 * \copyright       2016, Association de Robotique de Polytech Lille All rights reserved
 */


#ifndef _FINAL_APPROACH__ARTAGFA__H_
#define _FINAL_APPROACH__ARTAGFA__H_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <mutex>

typedef struct arTag_s
{
	int id;
	geometry_msgs::Pose pose;
	// Les deux champs qui suivent existent par commodité
	double yaw;
	float distance;
} arTag_t;

class ArTagFA
{
	public:
		ArTagFA();

		std::vector<int> getId();
		std::vector<float> getPositionX();
		std::vector<float> getPositionZ();
		std::vector<float> getOrientationZ();
		std::vector<float> getDistance();
		std::vector<arTag_t> getArTags();
		bool getFoundId(){return m_foundId;}
		ros::Time getStamp(void){ return m_stamp;}
		std::string getFrame(void){ return m_frame;}

		void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
		/**
		 * \brief      Indique si l'objet a bien détécté des arTags (dans la limite de son filtrage)
		 *
		 * \return     true if has arTags, false otherwise
		 */
		bool hasArTags(void) { return !m_arTags.empty();}

	private:
		ros::NodeHandle m_nh;
		ros::Subscriber m_arTagSub;
		std::vector<int> m_id;
		std::vector<float> m_positionX;
		std::vector<float> m_positionZ;
		std::vector<float> m_orientationZ;
		std::vector<float> m_distance;
		bool m_foundId;
		std::string m_frame;
		ros::Time m_stamp;
		std::vector<arTag_t> m_arTags;
		std::mutex m_mutex;
};

#endif // _FINAL_APPROACH__ARTAGFA__H_
