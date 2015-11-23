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
#include <iostream>


/**
 * \class State
 * \brief Classe abstraite contenant un état pour les algorithmes de recherche
 *
 * La classe contient un pointeur sur l'état précedent, ainsi que les coùts calculés
 *
 */
class State
{
public:
    State():m_cost(0), m_stepCost(0), m_prevState(nullptr)
    {

    }
    virtual ~State()
    {

    }

    void setPrevState(const std::shared_ptr<State> &prevState)
    {
        m_prevState = prevState;
    }
    const std::shared_ptr<State> &getPrevState() const
    {
        return m_prevState;
    }
    void setCost(double cost)
    {
        m_cost = cost;
    }
    double getCost() const
    {
        return m_cost;
    }
    void setStepCost(double cost)
    {
        m_stepCost = cost;
    }
    double getStepCost() const
    {
        return m_stepCost;
    }

    virtual bool compare(const State &s) const = 0;
    virtual std::size_t hash() const = 0;


    virtual std::ostream& toStream(std::ostream& os) const = 0;
    friend std::ostream& operator<<(std::ostream& os, State &state)
    {
        state.toStream(os);
        return os;
    }
protected:
    double m_cost, m_stepCost;
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

inline bool operator== (const State& lhs, const State& rhs)
{
    return lhs.compare(rhs);
}

inline bool operator!= (const State& lhs, const State& rhs)
{
    return !(lhs == rhs);
}

//std::size_t hash(const std::shared_ptr<State> &s);


/**
 * \class StateComparison
 * \brief Classe fournissant un moyen de comparer deux State, elle est utilisée dans les conteneurs priority_queue
 *
 *
 */
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

namespace std
{
    /**
     * \struct hash<std::shared_ptr<State>>
     * \brief Spécialisation de la structure hash, pour le type std::shared_ptr<State>, utilisée dans les unordered_set
     *
     *
     */
    template<>
    struct hash<std::shared_ptr<State>>
    {
        typedef std::shared_ptr<State> argument_type;
        typedef std::size_t result_type;

        result_type operator()(argument_type const& s) const
        {

            result_type const h (s->hash());
            //std::cout << "hash class : " << h << std::endl;
            return h;
        }
    };

    /**
     * \struct equal_to<std::shared_ptr<State>>
     * \brief Spécialisation de la structure equal_to, pour le type std::shared_ptr<State>, utilisée dans les unordered_set
     *
     *
     */
    template<>
    struct equal_to<std::shared_ptr<State>>
    {
        typedef std::shared_ptr<State> argument_type;

        bool operator() (argument_type const& x, argument_type const& y) const
        {
            return *x==*y;
        }
    };
}

#endif /* PATH_FINDER_STATE_H_ */
