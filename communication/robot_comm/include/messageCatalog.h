/**
 * \file         messageCatalog.h
 *
 * \brief
 *
 * \author       Coelen Vincent
 *               Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-05-22
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef MESSAGECATALOG_H
#define MESSAGECATALOG_H

#include <iostream>
#include <google/protobuf/message.h>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include <typeinfo>
#include <typeindex>

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "OrderRequest.pb.h"
#include "OrderResponse.pb.h"
#include "Map.pb.h"
#include "Obstacle.pb.h"
#include "Pose2D.pb.h"
#include "Time.pb.h"

class MessageCatalog
{
	private:
		typedef std::unordered_map<std::type_index, int> TypeIntMap;
		TypeIntMap m_catalog;

        typedef std::unordered_map<int, std::shared_ptr<google::protobuf::Message> (*)()> IntProtoMap;
        IntProtoMap m_catalogType;

	public:
		template<class T>
		void add();
		int serialize(std::vector<unsigned char>& buffer, std::shared_ptr<google::protobuf::Message>& msg);
		//template<class T>
		std::shared_ptr<google::protobuf::Message> deserialize(int code, const std::vector<unsigned char>& buffer);

		/* Constructeur */
		MessageCatalog()
		{ 
		}

		/* Destructeur */
		~MessageCatalog()
		{
		}
};

template<class T>
std::shared_ptr<google::protobuf::Message> createMsg()
{
    std::shared_ptr<google::protobuf::Message> msg(new T());
    return msg;
}

template<class T>
void MessageCatalog::add()
{
	const std::type_index key(typeid(T));
	T t;
	m_catalog[key] = t.code();
    m_catalogType[t.code()] = createMsg<T>;
}

#endif
