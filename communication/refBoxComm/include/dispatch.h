/**
 * \file 		dispatch.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-09
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef DISPATCH_H_
#define DISPATCH_H_

#include <unordered_map>
#include <typeinfo>
#include <exception>
#include <stdexcept>
#include <functional>

#include <typeindex>
//#include "typeInfo.h"


template <
	class Base,
	typename ResultType = void,
	typename CallbackType = ResultType (*)(Base&)
>
class StaticDispatch
{
private:
	typedef std::type_index KeyType;
	typedef CallbackType MappedType;
	typedef std::unordered_map<KeyType, MappedType> MapType;

	MapType m_callbackMap;
public:
	ResultType Go(Base& l)
	{
		typename MapType::iterator it = m_callbackMap.find(KeyType(typeid(l)));
		if(it == m_callbackMap.end())
		{
			throw std::runtime_error("Dispatch : Function not found");
		}
		return (it->second)(l);
	}

	template<class SomeClass>
	void Add(MappedType func)
	{
		const KeyType key(typeid(SomeClass));
		m_callbackMap[key] = func;
	}

};

#endif /* DISPATCH_H_ */
