/**
 * \file 		typeInfo.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2015-02-09
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef TYPEINFO_H_
#define TYPEINFO_H_

#include <typeinfo>
#include <stdio.h>
#include <stdlib.h> 
#include <string>

//From Modern C++ (A. Alexandrescu)
//replace by type_index, if using c++11
class TypeInfo
{
public:
	// Constructors/destructors
	TypeInfo() // needed for containers
	{
		class N {};
		pInfo_ = &typeid(N);
	}

	TypeInfo(const std::type_info& t):pInfo_(&t) {}
	//TypeInfo(const TypeInfo&);

	const std::type_info& Get() const
	{
		return *pInfo_;
	}

	TypeInfo& operator=(const TypeInfo&);
	// Compatibility functions
	bool before(const TypeInfo& t) const
	{
		return pInfo_->before(*t.pInfo_) != 0;
	}
	const char* name() const
	{
		return pInfo_->name();
	}
private:
	const std::type_info* pInfo_;

};

// Comparison operators
bool operator==(const TypeInfo& t1, const TypeInfo& t2);
bool operator!=(const TypeInfo& t1, const TypeInfo& t2);
bool operator<(const TypeInfo& t1, const TypeInfo& t2);
bool operator<=(const TypeInfo& t1, const TypeInfo& t2);
bool operator>(const TypeInfo& t1, const TypeInfo& t2);
bool operator>=(const TypeInfo& t1, const TypeInfo& t2);


/*
size_t hash<TypeInfo>::operator()(TypeInfo& t1) const
{
	return std::hash<std::string>()(t1.Get().name());
}
*/
template<class T> class TypeInfoHash;

template<>
class TypeInfoHash<TypeInfo> {
public:
    size_t operator()(const TypeInfo &t1) const
    {
    	return std::hash<std::string>()(t1.Get().name()); //TODO: moche à modifier
    }
};

#endif /* TYPEINFO_H_ */
