/**
 * \file        Polynome.h
 *
 * \brief
 *
 * \author       Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date         2016-06-29
 * \copyright    2016, Association de Robotique de Polytech Lille All rights reserved
 * \license
 * \version
 */

#ifndef COMMON_UTILS_POLYNOME_H_
#define COMMON_UTILS_POLYNOME_H_

#include <array>

namespace common_utils {

template<unsigned int N>
class Polynome
{
public:
    Polynome():m_coeffs{0}
    {

    }
    virtual ~Polynome()
    {

    }

    void setCoeff(unsigned int k, double coeff)
    {
        if (k>N)
        {
            return;
        }
        m_coeffs[N-k] = coeff;
    }

    double exec(double value)
    {
        double result = 0;
        for(auto &coeff: m_coeffs)
        {
            result = result*value + coeff;
        }
        return result;
    }

protected:
    std::array<double, N+1> m_coeffs;
};

} // namespace common_utils

#endif /* COMMON_UTILS_POLYNOME_H_ */
