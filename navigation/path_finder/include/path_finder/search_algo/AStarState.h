/**
 * \file 		AStarState.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-22
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#ifndef PATH_FINDER_ASTARSTATE_H_
#define PATH_FINDER_ASTARSTATE_H_

#include <memory>
#include "State.h"

/**
 * \class AStarState
 * \brief Classe dérivée de State ajoutant les paramêtres spécifique à l'algorithme Astar
 *
 *
 */
class AStarState : public State
{
public:
    AStarState() : State(), m_gCost(0)
    {

    }
    virtual ~AStarState()
    {

    }

    void setGCost(double cost)
    {
        m_gCost = cost;
    }
    double getGCost() const
    {
        return m_gCost;
    }

    virtual bool compare(const State &s) const override
    {
        return false;
    }
    virtual std::size_t hash() const override
    {
        return 0;
    }

    virtual std::ostream& toStream(std::ostream& os) const override
    {
        os << "AstarState, gCost : " << m_gCost;
    }

protected:
    double m_gCost;

};


#endif /* PATH_FINDER_ASTARSTATE_H_ */
