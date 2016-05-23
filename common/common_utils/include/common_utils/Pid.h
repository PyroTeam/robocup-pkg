/**
 * \file
 * \brief      Un simple PID, directement tiré du path_tracker
 * \author     Valentin Vergez (valentin.vergez@gmail.com)
 * \date       Create : 2016-05-22
 * \date       Last modified : 2016-05-22
 * \copyright  2016, Association de Robotique de Polytech Lille All rights
 *             reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of robocup-pkg nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
