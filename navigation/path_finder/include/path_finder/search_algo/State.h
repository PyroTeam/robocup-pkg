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

#include <memory>

class State
{
public:
    State():m_cost(0), m_prevState(nullptr)
    {

    }
    virtual ~State()
    {

    }

    void setPrevState(const std::shared_ptr<State> &prevState)
    {
        m_prevState = prevState;
    }

    void setCost(double cost)
    {
        m_cost = cost;
    }
    double getCost() const
    {
        return m_cost;
    }

    virtual std::ostream& toStream(std::ostream& os) const = 0;
    friend std::ostream& operator<<(std::ostream& os, State &state)
    {
        state.toStream(os);
        return os;
    }
protected:
    double m_cost;
    std::shared_ptr<State> m_prevState;
};

inline bool operator< (const State& lhs, const State& rhs)
{
    return lhs.getCost() < rhs.getCost();
}

inline bool operator> (const State& lhs, const State& rhs)
{
    return rhs < lhs;
}

class StateComparison
{
public:
    StateComparison(const bool& reverse=false)
    {
        m_reverse=reverse;
    }
    bool operator() (const std::shared_ptr<State> &lhs, const std::shared_ptr<State> &rhs)
    {
        if (m_reverse)
        {
            return (*lhs>*rhs);
        }
        else
        {
            return (*lhs<*rhs);
        }
    }
protected:
    bool m_reverse;
};

#endif /* PATH_FINDER_STATE_H_ */
