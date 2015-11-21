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

class Heuristic
{
public:
    Heuristic()
    {

    }
    virtual ~Heuristic()
    {

    }

    virtual double evaluate() = 0;
    virtual double operator()() = 0;
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

    virtual double evaluate() override
    {

    }
    virtual double operator()() override
    {
        return evaluate();
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

    virtual double evaluate() override
    {

    }
    virtual double operator()() override
    {
        return evaluate();
    }
};

#endif /* PATH_FINDER_HEURISTIC_H_ */
