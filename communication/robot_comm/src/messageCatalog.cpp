/**
 * \file         messageCatalog.cpp
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

#include "messageCatalog.h"

class MessageCatalog;

int MessageCatalog::serialize(std::vector<unsigned char>& buffer, std::shared_ptr<google::protobuf::Message>& msg)
{
	buffer.resize(128, 0);
    msg->SerializeToArray((void*)buffer.data(), buffer.size());
	buffer.resize(msg->ByteSize());
  
	std::type_index key(typeid(*msg));

	typename TypeIntMap::iterator it = m_catalog.find(std::type_index(typeid(*msg)));
	if(it == m_catalog.end())
	{
		std::cout << "Clé non trouvée" << std::endl;;
		return -1;
	}
	else
	{
		int code = m_catalog[key];
		return code;
	}
}

std::shared_ptr<google::protobuf::Message> MessageCatalog::deserialize(int code, const std::vector<unsigned char>& buffer)
{
	typename IntProtoMap::iterator it = m_catalogType.find(code);
	if(it == m_catalogType.end())
	{
		std::cout << "Code non trouvé" << std::endl;;
		return nullptr;
	}
	else
	{
		std::shared_ptr<google::protobuf::Message> msg = m_catalogType[code]();
   		msg->ParseFromArray((void*)buffer.data(), buffer.size());
    	return msg;
	}
}
