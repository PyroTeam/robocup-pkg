/**
 * \file
 * \brief      Un simple PID, directement tiré du path_tracker
 * \author     Valentin Vergez (valentin.vergez@gmail.com)
 * \date       Create : 2016-05-22
 * \date       Last modified : 2016-05-22
 * \copyright  2016, Association de Robotique de Polytech Lille All rights reserved
 */

#ifndef _COMMON_UTILS__PID__H_
#define _COMMON_UTILS__PID__H_

/**
 * \brief      Simple PID
 *
 * \NOTE: Si on accepte de lier Pid à ROS, il faudrait utiliser ros::Time pour la
 * période en s'assurant que ce soit compatible avec la simulation et la
 * synchronisation temporelle). On pourrait aussi passer le temps en argument à
 * update.
 *
 * \NOTE: L'ajout de l'anti windup serait un gros plus.
 */
class Pid
{
public:
	Pid(float Kp, float Ki, float Kd, float T);
	~Pid();
	/**
	 * \brief      Calcul la commande à appliquer
	 *
	 * \param[in]  err   L'erreur
	 *
	 * \return     La commande
	 */
	float update(float err);
	void setKp(float Kp) { m_Kp = Kp;};
	void setKi(float Ki) { m_Ki = Ki;};
	void setKd(float Kd) { m_Kd = Kd;};

private:
	float m_Kp;   // Coeff P
	float m_Ki;   // Coeff I
	float m_Kd;   // Coeff D
	float m_err;  // Erreur précédente
	float m_I;    // Intégrale de l'erreur
	float m_T;    // Période entre deux appels de update
};

#endif // _COMMON_UTILS__PID__H_
