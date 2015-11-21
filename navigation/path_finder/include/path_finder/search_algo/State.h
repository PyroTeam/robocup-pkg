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
    State()
    {

    }
    virtual ~State()
    {
        
    }
protected:
    double m_cost;
};

#endif /* PATH_FINDER_STATE_H_ */
