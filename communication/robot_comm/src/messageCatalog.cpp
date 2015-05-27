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
	std::cout << std::dec << buffer.size() << std::endl;
    /*bool result = */msg->SerializeToArray((void*)buffer.data(), buffer.size());
	buffer.resize(msg->ByteSize());
	std::cout << std::dec << msg->ByteSize() << std::endl;
	std::cout << std::dec << buffer.size() << std::endl;
    /*if (!result)
    {
        return 0;
    }
	else
	{*/
		std::type_index key(typeid(*msg));
		int code = m_catalog[key];
		return code;
	//}
}

std::shared_ptr<google::protobuf::Message> MessageCatalog::deserialize(int code, const std::vector<unsigned char>& buffer)
{
	std::shared_ptr<google::protobuf::Message> msg = m_catalogType[code]();
    msg->ParseFromArray((void*)buffer.data(), buffer.size());
    return msg;
}
