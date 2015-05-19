/**
 * \file         udpPeer.cpp
 *
 * \brief
 *
 * \author       Coelen Vincent
 *               Tissot Elise (elise-tissot@polytech-lille.net)
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

#include "udpPeer.h"
//#include "encryptUtils.h"

//using boost::asio::ip::udp;

UdpPeer::UdpPeer(boost::asio::io_service& io_service, int port):
m_port(port), m_socket(io_service, udp::endpoint(udp::v4(), m_port))
{
	m_broadcastEndpoint = udp::endpoint(boost::asio::ip::address::from_string("192.168.0.255"), m_port);
	m_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
	m_socket.set_option(udp::socket::broadcast(true));
	udp::endpoint(udp::v4(), m_port),
	m_socket.async_receive_from(boost::asio::buffer(m_bufferRecv),
	boost::bind(&UdpPeer::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

/* Sert à envoyer le paquet */
void UdpPeer::send(std::shared_ptr<google::protobuf::Message>& msg)
{
	std::vector<unsigned char> m_buffer= {'H','e','l','l','o'};
	std::vector<unsigned char>::iterator it;
	it = m_buffer.begin();
	//unsigned char code = m_msgCatalog.serialize(m_buffer, msg);
	m_buffer.insert(it, 'c');
	std::vector<unsigned char> m_buffer_s;
	std::vector<unsigned char> IV;
	m_encryptUtil.encrypt(m_buffer, m_buffer_s, IV);
	it = m_buffer_s.begin();
	m_buffer_s.insert(it, IV.begin(), IV.end());
	it = m_buffer_s.begin();
	m_buffer_s.insert(it, '1');
	//std::string code = "code";
	it = m_buffer_s.begin();
	m_buffer_s.insert(it, 'c');

	m_socket.async_send_to(boost::asio::buffer(m_buffer_s), m_broadcastEndpoint,
        boost::bind(&UdpPeer::handle_send, this, &m_buffer_s,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void UdpPeer::setCrypto(std::vector<unsigned char> key, EncryptUtils::CIPHER_TYPE cipher)
{
	m_encryptUtil.setConfig(key, cipher);
}

void UdpPeer::handle_receive(const boost::system::error_code &error, std::size_t size)
{
	std::cout << "Entree receive (message recu)" << std::endl;
	if (!error || error == boost::asio::error::message_size)
	{
		if (m_bufferRecv[0] != 'c' /* code */)
		{
			return;
		}
		else
		{
			std::vector<unsigned char>::iterator it;
			std::vector<unsigned char>::iterator it_fin;
			it_fin = m_bufferRecv.begin()+17;
			if (m_bufferRecv[1] == '0') /* Le message n'est pas crypté */
			{ 
				std::cout << "Message non crypte recu" << std::endl;
				it = m_bufferRecv.begin();
				m_bufferRecv.erase(it, it_fin);
				unsigned char code = m_bufferRecv[0];
				m_bufferRecv.erase(it);
				//google::protobuf::Message* msg = m_msgCatalog.deserialize(code, m_buffer);
				//m_msgDispatcher.go(msg);
			}
			else /* m_buffer == '1' */
			{
				std::cout << "Message crypte recu" << std::endl;
				std::vector<unsigned char> IV(m_bufferRecv.begin()+2, m_bufferRecv.begin()+18);
				m_bufferRecv.erase(it, it_fin);
				std::vector<unsigned char> m_buffer_s;
				m_encryptUtil.decrypt(m_bufferRecv, m_buffer_s, IV);
				std::cout << "Buffer recu : [";
    			for(auto &i: m_buffer_s)
   				{
        			std::cout << i;
    			}
    			std::cout << "]" << std::endl;
				unsigned char code = m_buffer_s[0];
				m_buffer_s.erase(it);
				//google::protobuf::Message* msg = m_msgCatalog.deserialize(code, m_buffer_s);
				//m_msgDispatcher.go(msg);
			}
		}
	}
}

void UdpPeer::handle_send(std::vector<unsigned char>* /*message*/,
	const boost::system::error_code& /*error*/,
        std::size_t /*bytes_transferred*/)
  	{
  	}

void UdpPeer::registerMessage()
{

}
