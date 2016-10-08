/**
 * \file 		Chrono.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-29
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef COMMON_UTILS_CHRONO_H_
#define COMMON_UTILS_CHRONO_H_

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

namespace common_utils {


/**
 * \class Chrono
 * \brief Classe utilitaire pour mesurer un intervalle de temps
 *
 * Utile pour mesurer le temps passé dans un algorithme par exemple
 *
 */
template<class Clock>
class Chrono
{
public:
    Chrono()
    {

    }
    ~Chrono()
    {

    }

    const typename Clock::time_point &start()
    {
        m_start = Clock::now();
        return m_start;
    }
    const typename Clock::time_point &stop()
    {
        m_stop = Clock::now();
        return m_stop;
    }
    double duration()
    {
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(m_stop - m_start);
        return time_span.count();
    }
    friend std::ostream& operator<<(std::ostream& os, Chrono<Clock> &chrono)
    {
        os << chrono.duration() << " seconds";
        return os;
    }

protected:
    typename Clock::time_point m_start;
    typename Clock::time_point m_stop;
};

typedef Chrono<std::chrono::high_resolution_clock> HighResChrono;
typedef Chrono<std::chrono::system_clock> SystemChrono;
typedef Chrono<std::chrono::steady_clock> SteadyChrono;


} // namespace common_utils

#endif /* COMMON_UTILS_CHRONO_H_ */
