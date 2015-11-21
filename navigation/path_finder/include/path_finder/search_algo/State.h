/**
 * \file 		State.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-21
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#ifndef PATH_FINDER_STATE_H_
#define PATH_FINDER_STATE_H_

class State
{
public:
    State():m_cost(0)
    {

    }
    virtual ~State()
    {

    }

    virtual std::ostream& toStream(std::ostream& os) = 0;
    friend std::ostream& operator<<(std::ostream& os, State &state)
    {
        state.toStream(os);
        return os;
    }
protected:
    double m_cost;
};

#endif /* PATH_FINDER_STATE_H_ */
