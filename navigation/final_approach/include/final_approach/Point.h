/**
 * \file
 * \class			Point
 * \brief			classe représentant un point du laser
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-20
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef _FINAL_APPROACH__POINT__H_
#define _FINAL_APPROACH__POINT__H_

#include <cmath>

class Point
{
	public:
		Point(float r = 0.0, double phi = 0.0);
		~Point();

		void setR(float val){m_r = val;}
		void setPhi(double val){m_phi = val;}

		float getR() {return m_r;}
		double getPhi() {return m_phi;}
		float getX() {return m_r*sin(M_PI_2 - m_phi);}
		float getY() {return m_r*cos(M_PI_2 - m_phi);}

	private:
		float m_r;
		double m_phi;
};

#endif	// _FINAL_APPROACH__POINT__H_
