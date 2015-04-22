#ifndef UDPPEER_H
#define UDPPEER_H

#include <iostream>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <string>
#include <list>
#include <vector>

#include "encryptUtils.h"

#define CODE 0xff

using boost::asio::ip::udp;
//using google::protobuf;

//class MessageDispatcher;
//class MessageCatalog;
class EncryptUtils;
//class EntryPoint;

class UdpPeer{
	private:
	boost::asio::ip::udp::socket m_socket;
	boost::asio::ip::udp::endpoint m_broadcastEndpoint;
//	MessageCatalog m_msgCatalog;
//	MessageDispatcher m_msgDispatcher;
	bool m_isCrypto;
	EncryptUtils m_encryptUtil;
//	std::list<EntryPoint *> m_ep;
	std::vector<unsigned char> m_buffer_recv;
	int m_port;

	void handle_receive(const boost::system::error_code &error, std::size_t size);
	void handle_send(std::vector<unsigned char>*, const boost::system::error_code&, std::size_t);

	public:
	void send(std::shared_ptr<google::protobuf::Message>& msg);
	void setCrypto(std::vector<unsigned char> key, EncryptUtils::CIPHER_TYPE cipher);
	void registerMessage();

	UdpPeer(boost::asio::io_service& io_service, int port);
};

#endif
