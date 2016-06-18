/**
 * \file 		Heuristic.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-18
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef PATH_FINDER_HEURISTIC_H_
#define PATH_FINDER_HEURISTIC_H_

#include <cmath>
#include "search_algo/State.h"
#include "search_algo/PointState.h"

/**
 * \class Heuristic
 * \brief Classe abstraite pour le calcul d'une heuristique
 *
 * La classe offre les interfaces génériques de calcul d'une heuristique
 * pour les algorithme de recherche utilisant ce principe (tel que Astar).
 *
 */
class Heuristic
{
public:
    Heuristic()
    {

    }
    virtual ~Heuristic()
    {

    }

    virtual double evaluate(const State &startState, const State &endState) = 0;
    virtual double operator()(const State &startState, const State &endState) = 0;
};

/**
 * \class EuclidianHeuristic
 * \brief Classe concrète dérivée de Heuristic pour l'heuristique euclidienne
 *
 * L'heuristique euclidienne représente le calcul de la distance entre un point
 * du graphe et le point de d'arrivée de la recherche
 *
 */
class EuclidianHeuristic : public Heuristic
{
public:
    EuclidianHeuristic():Heuristic()
    {

    }
    virtual ~EuclidianHeuristic()
    {

    }

    virtual double evaluate(const State &startState, const State &endState) override
    {
        //TODO généraliser ce code
        //version pour les tests
        //fonctionne uniquement avec des PointState
        const PointState &psStart = dynamic_cast<const PointState&>(startState);
        const PointState &psEnd   = dynamic_cast<const PointState&>(endState);
        double x2 = (psStart.get().x - psEnd.get().x)*(psStart.get().x - psEnd.get().x);
        double y2 = (psStart.get().y - psEnd.get().y)*(psStart.get().y - psEnd.get().y);
        return sqrt(x2 + y2);
    }
    virtual double operator()(const State &startState, const State &endState) override
    {
        return evaluate(startState, endState);
    }
};

/**
 * \class DiagonalHeuristic
 * \brief Classe concrète dérivée de Heuristic pour l'heuristique diagonale
 *
 * L'heuristique diagonal s'appelle distance de Chebyshev si m_d = 1 et m_2 = 1
 * et distance octile si m_d = 1 et m_2 = sqrt(2)
 *
 */
class DiagonalHeuristic : public Heuristic
{
public:
    DiagonalHeuristic(double d=1.0, double d2 = sqrt(2.0)):Heuristic(),m_d(d),m_d2(d2)
    {

    }
    virtual ~DiagonalHeuristic()
    {

    }

    virtual double evaluate(const State &startState, const State &endState) override
    {
        //TODO généraliser ce code
        //version pour les tests
        //fonctionne uniquement avec des PointState
        const PointState &psStart = dynamic_cast<const PointState&>(startState);
        const PointState &psEnd   = dynamic_cast<const PointState&>(endState);
        double dx = std::abs(psStart.get().x - psEnd.get().x);
        double dy = std::abs(psStart.get().y - psEnd.get().y);
        return (m_d * (dx+dy) + (m_d2 -2*m_d) * std::min(dx, dy));
    }
    virtual double operator()(const State &startState, const State &endState) override
    {
        return evaluate(startState, endState);
    }
protected:
    double m_d, m_d2;
};

/**
 * \class ManhattanHeuristic
 * \brief Classe concrète dérivée de Heuristic pour l'heuristique manhattan
 *
 *
 */
class ManhattanHeuristic : public Heuristic
{
public:
    ManhattanHeuristic():Heuristic()
    {

    }
    virtual ~ManhattanHeuristic()
    {

    }

    virtual double evaluate(const State &startState, const State &endState) override
    {

    }
    virtual double operator()(const State &startState, const State &endState) override
    {
        return evaluate(startState, endState);
    }
};

#endif /* PATH_FINDER_HEURISTIC_H_ */
