/**
 * \file         udpPeer.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent
 *               Tissot Elise (elise.tissot@polytech-lille.net)
 * \date         2015-04-06
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include <iostream>
#include <string>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <initializer_list>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>

#include "udpPeer.h"

/* Constantes */
#define TAILLE_BUFFER 64
#define CRYPTE         1
#define NON_CRYPTE     0
#define CODE          'c'
#define TAILLE_HEADER 18
#define DEBUT_IV       2

UdpPeer::UdpPeer(boost::asio::io_service& io_service, int portIn, int portOut, std::string adresseIP):
m_portIn(portIn), m_portOut(portOut), m_socket(io_service), m_adresseIP(adresseIP)
{
	m_socket.open(boost::asio::ip::udp::v4());
	boost::asio::socket_base::broadcast option(true);
	m_socket.set_option(option);
	m_broadcastEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(m_adresseIP), m_portOut);
	m_bufferRecv.resize(TAILLE_BUFFER, 0);
	m_remoteEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), m_portIn);
	m_socket.bind(m_remoteEndpoint);
	startReceive();

	// Get all ip address
	struct ifaddrs *ifap, *ifa;
	struct sockaddr_in *sa;
	char *addr;

	getifaddrs (&ifap);
	for (ifa = ifap; ifa; ifa = ifa->ifa_next)
	{
		if (ifa->ifa_addr->sa_family==AF_INET)
		{
			sa = (struct sockaddr_in *) ifa->ifa_addr;
			addr = inet_ntoa(sa->sin_addr);
			boost::asio::ip::udp::endpoint ep = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(addr), m_socket.local_endpoint().port());
			m_localEndPoints.push_back(ep);
			printf("Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
		}
	}
	freeifaddrs(ifap);
}


bool UdpPeer::isLocalEndpoint(boost::asio::ip::udp::endpoint ep)
{
	for(auto &localEp : m_localEndPoints)
	{
		if(ep == localEp)
		{
			return true;
		}
	}
	return false;
}

void UdpPeer::startReceive()
{
	m_bufferRecv.resize(TAILLE_BUFFER, 0);
	m_socket.async_receive_from(boost::asio::buffer(m_bufferRecv), m_remoteEndpoint,
		boost::bind(&UdpPeer::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpPeer::send(std::shared_ptr<google::protobuf::Message>& msg)
{
	std::vector<unsigned char> m_buffer_tmp;
	std::vector<unsigned char>::iterator it;

	int code = m_msgCatalog->serialize(m_buffer_tmp, msg);
	it = m_buffer_tmp.begin();
	m_buffer_tmp.insert(it, code); /* code sérialisation */
	std::vector<unsigned char> IV;
	m_encryptUtil.encrypt(m_buffer_tmp, m_buffer, IV);
	it = m_buffer.begin();
	m_buffer.insert(it, IV.begin(), IV.end());
	it = m_buffer.begin();
	m_buffer.insert(it, CRYPTE);
	it = m_buffer.begin();
	m_buffer.insert(it, CODE);

	m_socket.async_send_to(boost::asio::buffer(m_buffer), m_broadcastEndpoint,
        boost::bind(&UdpPeer::handle_send, this, &m_buffer,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

/*void UdpPeer::setCrypto(std::vector<unsigned char> key, EncryptUtils::CIPHER_TYPE cipher)
{
	m_encryptUtil.setConfig(key, cipher);
}*/

void UdpPeer::handle_receive(const boost::system::error_code &error, std::size_t size)
{
	// Get remote endpoint
	boost::asio::ip::address remote_ad = m_remoteEndpoint.address();
	std::string s = remote_ad.to_string();
	std::cout << "remote endpoint : " << s << std::endl;

	// Discard message if remote endpoint is a local endpoint
	if (isLocalEndpoint(m_remoteEndpoint))
	{
		std::cout << "Receive my own messages !!" << std::endl;
		startReceive();
		return;
	}

	if (!error || error == boost::asio::error::message_size)
	{
		if (m_bufferRecv[0] != CODE)
		{
			startReceive();
			return;
		}
		else
		{
			std::vector<unsigned char>::iterator it;
			std::vector<unsigned char>::iterator it_fin;
			it_fin = m_bufferRecv.begin()+TAILLE_HEADER;
			if (m_bufferRecv[1] == NON_CRYPTE)
			{
				it = m_bufferRecv.begin();
				m_bufferRecv.erase(it, it_fin);
				unsigned char code = m_bufferRecv[0];
				m_bufferRecv.erase(it);
				std::shared_ptr<google::protobuf::Message> msg = m_msgCatalog->deserialize(code, m_buffer);
				m_msgDispatcher->Go(*msg);
			}
			else /* m_buffer == CRYPTE */
			{
				it = m_bufferRecv.begin();
				std::vector<unsigned char> IV(m_bufferRecv.begin()+DEBUT_IV, m_bufferRecv.begin()+TAILLE_HEADER);
				m_bufferRecv.erase(it, it_fin);
				m_bufferRecv.resize(size-TAILLE_HEADER);
				std::vector<unsigned char> buffer_s;
				m_encryptUtil.decrypt(m_bufferRecv, buffer_s, IV);
				int code = buffer_s[0];
				it = buffer_s.begin();
				buffer_s.erase(it);
				std::shared_ptr<google::protobuf::Message> msg = m_msgCatalog->deserialize(code, buffer_s);
				/*std::shared_ptr<Activity> msgActivity = std::dynamic_pointer_cast<Activity>(msg);
				std::cout << msgActivity->name() << std::endl;*/
				m_msgDispatcher->Go(*msg);
			}
		}
	}

	startReceive();
}

void UdpPeer::handle_send(std::vector<unsigned char>* /*message*/,
	const boost::system::error_code& /*error*/,
        std::size_t size/*bytes_transferred*/)
  	{
  	}

void UdpPeer::setDispatcher(std::shared_ptr<MessageDispatcher> dispatcher)
{
	m_msgDispatcher = dispatcher;
}

void UdpPeer::setCatalog(std::shared_ptr<MessageCatalog> catalog)
{
	m_msgCatalog = catalog;
}
