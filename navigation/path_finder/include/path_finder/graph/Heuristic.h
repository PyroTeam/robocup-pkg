/**
 * \file 		Heuristic.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef PATH_FINDER_HEURISTIC_H_
#define PATH_FINDER_HEURISTIC_H_

#include "search_algo/State.h"

class Heuristic
{
public:
    Heuristic()
    {

    }
    virtual ~Heuristic()
    {

    }

    virtual double evaluate(State &startState, State &endState) = 0;
    virtual double operator()(State &startState, State &endState) = 0;
};

class EuclidianHeuristic : public Heuristic
{
public:
    EuclidianHeuristic():Heuristic()
    {

    }
    virtual ~EuclidianHeuristic()
    {

    }

    virtual double evaluate(State &startState, State &endState) override
    {

    }
    virtual double operator()(State &startState, State &endState) override
    {
        return evaluate(startState, endState);
    }
};

class ManhattanHeuristic : public Heuristic
{
public:
    ManhattanHeuristic():Heuristic()
    {

    }
    virtual ~ManhattanHeuristic()
    {

    }

    virtual double evaluate(State &startState, State &endState) override
    {

    }
    virtual double operator()(State &startState, State &endState) override
    {
        return evaluate(startState, endState);
    }
};

#endif /* PATH_FINDER_HEURISTIC_H_ */
