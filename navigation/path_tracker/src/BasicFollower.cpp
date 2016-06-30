/**
 * \file         BasicFollower.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-23
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#include <memory>
#include "path_tracker/BasicFollower.h"
#include "geometry_utils/geometry_utils.h"
#include "common_utils/RateLimiter.h"
#include "common_utils/controller/PidWithAntiWindUp.h"
#include "common_utils/Polynome.h"

const double EPSILON = 0.05;
const double ADVANCE_WINDOW = 0.8;

//vitesse min robotino fixe
//TODO: à parametrer
const float VminStatic = 0.05;
const float VmaxUp = 0.3;
const float VmaxLow = 0.05;
const float angleMin = 5.0/180.0 * M_PI;


geometry_msgs::Twist BasicFollower::generateNewSetpoint()
{

    float Vmax = m_Vmax;
    float Vlim = m_Vmax;

    geometry_msgs::Twist twist;
    if(m_status != PathFollowerStatus_t::IN_PROGRESS || m_pathSize == 0)
    {
        ROS_INFO("Trajectory pb");
        return twist;
    }

    //Obtenir la Pose dans le repère map
    const geometry_msgs::Pose2D &pose = m_robotPose.getPose2D();


    m_pathError = 0;
    float u = 10.0;//valeur supérieure à 1
    float delta_x = 0.0;
    float delta_y = 0.0;

    if (m_path.poses.size() > 1)
    {
        if(m_currentSegment < m_path.poses.size() - 1)
        {
            // recherche du segment a suivre
            while (u > 1.0 && m_currentSegment < m_path.poses.size() - 1)
            {
                delta_x = m_path.poses[m_currentSegment+1].pose.position.x - m_path.poses[m_currentSegment].pose.position.x;
                delta_y = m_path.poses[m_currentSegment+1].pose.position.y - m_path.poses[m_currentSegment].pose.position.y;

                float Rx = pose.x - m_path.poses[m_currentSegment].pose.position.x;
                float Ry = pose.y - m_path.poses[m_currentSegment].pose.position.y;

                float denom = (delta_x*delta_x + delta_y*delta_y);
                u = (Rx*delta_x + Ry*delta_y) / denom;

                if (u > 1.0)
                {
                    m_currentSegment++;
                }

                m_pathError = (Ry*delta_x - Rx*delta_y) / denom;
            }

            //calcul de l'orientation du segment
            float segmentAngle = atan2(delta_y, delta_x);

            //calcul de l'erreur en angle
            float errOri = 0.0;
            if (m_currentSegment >= m_path.poses.size()-2)
            {
                 //sur le dernier segment on commence a asservir sur l'orientation finale
                float lastYaw = tf::getYaw(m_path.poses.back().pose.orientation);
                errOri = geometry_utils::normalizeAngle(lastYaw - pose.theta);
            }
            else
            {
                errOri = geometry_utils::normalizeAngle(segmentAngle - pose.theta);
            }

            //mise à jour de la consigne d'angle
            twist.angular.z = m_controllerOri->update(errOri);

            //estimation de la vitesse max
            //On limite la vitesse max si l'ecart d'angle est trop grand
            if (std::abs(errOri) < angleMin)
            {
                Vmax = VmaxUp;
            }
            else
            {
                common_utils::Polynome<1> droite;
                float a = (VmaxUp - VmaxLow)/(angleMin - M_PI);
                droite.setCoeff(1, a);
                droite.setCoeff(0, VmaxUp - a*angleMin);
                Vmax = droite.exec(std::abs(errOri));
            }


            //parcours du chemin pour calcul amax et damax
            int tmpIndex = m_currentSegment+1;
            float dWindow = ADVANCE_WINDOW;
            float dCurrent = 0;
            while (tmpIndex < m_path.poses.size()-1 && dCurrent < dWindow)
            {
                float dx = m_path.poses[tmpIndex+1].pose.position.x - m_path.poses[tmpIndex].pose.position.x;
                float dy = m_path.poses[tmpIndex+1].pose.position.y - m_path.poses[tmpIndex].pose.position.y;
                float dSegment = sqrt(dx*dx + dy*dy);
                dCurrent += dSegment;
                tmpIndex++;
            }
            float Vmin = VminStatic;
            if (tmpIndex == m_path.poses.size()-1)
            {
                Vlim = m_speedLimiter.update((Vmax - Vmin)/dWindow * dCurrent + Vmin);
            }
            else
            {
                Vlim = m_speedLimiter.update(Vmax);
            }
            //En repere segment local
            if (std::shared_ptr<common_utils::PidWithAntiWindUp> p = std::dynamic_pointer_cast<common_utils::PidWithAntiWindUp>(m_controllerVel))
            {
                p->setLimits(-Vlim, Vlim);
            }
            float Vy = m_controllerVel->update(-m_pathError);

            float Vx = sqrt(Vlim*Vlim - Vy*Vy);
            //Passer le vecteur (Vx Vy) en repere robot pour appliquer a cmdVel
            //TODO: utiliser la transformation de common utils
            float theta = segmentAngle - pose.theta;
            twist.linear.x = Vx * cos(theta) - Vy * sin(theta);
            twist.linear.y = Vx * sin(theta) + Vy * cos(theta);
        }
        else if (m_currentSegment >= m_path.poses.size()-1)
        {
            //arreter robot
            twist.linear.x = 0;
            twist.linear.y = 0;
            //orientation finale
            float lastYaw = tf::getYaw(m_path.poses.back().pose.orientation);
            float errOrie = geometry_utils::normalizeAngle(lastYaw - pose.theta);
            if (std::abs(errOrie) < EPSILON)
            {
                m_status = PathFollowerStatus_t::TRAJ_END;
            }
            else
            {
                twist.angular.z = m_controllerOri->update(errOrie);
            }
        }
    }
    return twist;

}
