/**
 * \file 		typeInfo.cpp
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-03-01
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "typeInfo.h"

// Comparison operators
bool operator==(const TypeInfo& t1, const TypeInfo& t2)
{
	return (t1.Get() == t2.Get());
}
bool operator!=(const TypeInfo& t1, const TypeInfo& t2)
{
	return (t1.Get() != t2.Get());
}
bool operator<(const TypeInfo& t1, const TypeInfo& t2)
{
	return t1.Get().before( t2.Get() );
}
bool operator<=(const TypeInfo& t1, const TypeInfo& t2)
{
	return !(t2 < t1);
}
bool operator>(const TypeInfo& t1, const TypeInfo& t2)
{
	return t2 < t1;
}
bool operator>=(const TypeInfo& t1, const TypeInfo& t2)
{
	return !(t1 < t2);
}
