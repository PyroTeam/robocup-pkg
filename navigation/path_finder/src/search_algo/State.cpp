/**
 * \file 		State.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-11-22
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "search_algo/State.h"

std::size_t hash(const std::shared_ptr<State> &s)
{
    return s->hash();
}
