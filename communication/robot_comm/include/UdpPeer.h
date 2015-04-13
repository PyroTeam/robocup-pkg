#ifndef UDPPEER_H
#define UDPPEER_H

#include <iostream>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <string>
#include <list>

#include "encryptUtils.h"

#define CODE 0xff

using boost::asio::ip::udp;
//using google::protobuf;

//class MessageDispatcher;
//class MessageCatalog;
class encryptUtils;
//class EntryPoint;

class UdpPeer{
	private:
	boost::asio::ip::udp::socket m_socket;
	boost::asio::ip::udp::endpoint m_broadcastEndpoint;
//	MessageCatalog m_msgCatalog;
//	MessageDispatcher m_msgDispatcher;
	bool m_isCrypto;
//	encryptUtils m_encryptUtil;
//	std::list<EntryPoint *> m_ep;
	std::vector<unsigned char> m_buffer_recv;

	void handle_receive(const boost::system::error_code &error, std::size_t size);

	public:
	void send(google::protobuf::Message* msg, const boost::system::error_code&, std::size_t);
	void setCrypto(std::string key, std::string cipher);
	void registerMessage();	
};

#endif
