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

#include "udpPeer.h"

//using boost::asio::ip::udp;

UdpPeer::UdpPeer(boost::asio::io_service& io_service, int portIn, int portOut):
m_portIn(portIn), m_portOut(portOut), m_socket(io_service)
{
	m_socket.open(boost::asio::ip::udp::v4());
	boost::asio::socket_base::broadcast option(true);
	m_socket.set_option(option);
	m_broadcastEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("127.255.255.255"), m_portOut);
	m_bufferRecv.resize(64, 0);
	m_remoteEndpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), m_portIn);
	m_socket.bind(m_remoteEndpoint);
	m_socket.async_receive_from(boost::asio::buffer(m_bufferRecv), m_remoteEndpoint, 
		boost::bind(&UdpPeer::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpPeer::send(std::shared_ptr<google::protobuf::Message>& msg)
{
	std::vector<unsigned char> m_buffer_tmp;/*= {'H','e','l','l','o'};*/
	std::vector<unsigned char>::iterator it;
	std::cout << "Test" << std::endl;

	m_msgCatalog.add<Activity>();
	std::shared_ptr<Activity> activity(new Activity());

	activity->set_code(1);
	activity->set_name("Status");
	activity->set_nb_robot(1);
	activity->set_state(Activity_STATE_ROBOT_END);
	activity->set_machine_used(Activity_MACHINE_TYPE_BS);
	activity->set_nb_order(3);

	msg = activity;

	int code = m_msgCatalog.serialize(m_buffer_tmp, msg);

	std::cout << "Message serialise : ";
	for (auto &i : m_buffer_tmp)
	{
		std::cout << " " << std::hex << std::uppercase << int(i);
	}

	std::cout << std::endl;
	std::cout << "Code : " << code << std::endl;
	it = m_buffer_tmp.begin();
	m_buffer_tmp.insert(it, code); /* code sérialisation */
	std::cout << "Test" << std::endl;
	std::vector<unsigned char> IV;
	m_encryptUtil.encrypt(m_buffer_tmp, m_buffer, IV);

	std::cout << "Message crypte : ";
	for (auto &i : m_buffer)
	{
		std::cout << " " << std::hex << std::uppercase << int(i);
	}
	std::cout << std::endl;

	std::cout << "Vecteur IV : ";
	for (auto &i : IV)
	{
		std::cout << " " << std::hex << std::uppercase << int(i);
	}
	std::cout << std::endl;

	it = m_buffer.begin();
	m_buffer.insert(it, IV.begin(), IV.end());
	it = m_buffer.begin();
	m_buffer.insert(it, '1');
	//std::string code = "code";
	it = m_buffer.begin();
	m_buffer.insert(it, 'c');

	std::cout << "Buffer envoye : ";
	for (auto &i : m_buffer)
	{
		std::cout << " " << std::hex << std::uppercase << int(i);
	}
	std::cout << std::endl;

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
	std::cout << "Entree receive (message recu)" << std::endl;
	std::cout << "Octets transférés : " << std::dec << size << std::endl;
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
			it_fin = m_bufferRecv.begin()+18;
			std::cout << "Crypte ou non ? : " << m_bufferRecv[1] << std::endl;
			if (m_bufferRecv[1] == '0') /* Le message n'est pas crypté */
			{ 
				std::cout << "Message non crypte recu" << std::endl;
				it = m_bufferRecv.begin();
				m_bufferRecv.erase(it, it_fin);
				unsigned char code = m_bufferRecv[0];
				m_bufferRecv.erase(it);
				std::shared_ptr<google::protobuf::Message> msg = m_msgCatalog.deserialize(code, m_buffer);
				//m_msgDispatcher.go(msg);
			}
			else /* m_buffer == '1' */
			{
				std::cout << "Message crypte recu" << std::endl;
				it = m_bufferRecv.begin();
				std::vector<unsigned char> IV(m_bufferRecv.begin()+2, m_bufferRecv.begin()+18);

				std::cout << "IV : ";
				for (auto &i : IV)
				{
					std::cout << " " << std::hex << std::uppercase << int(i);
				}
				std::cout << std::endl;

				m_bufferRecv.erase(it, it_fin);

				m_bufferRecv.resize(size-18);

				std::cout << "Buffer a decrypter : ";
				for (auto &i : m_bufferRecv)
				{
					std::cout << " " << std::hex << std::uppercase << int(i);
				}
				std::cout << std::endl;

				std::vector<unsigned char> buffer_s;
				//m_bufferRecv.resize(size-18);
				m_encryptUtil.decrypt(m_bufferRecv, buffer_s, IV);
				std::cout << "Buffer recu : [";
    			for(auto &i: buffer_s)
   				{
        			std::cout << " " << std::hex << std::uppercase << int(i);
    			}
    			std::cout << "]" << std::endl;
				int code = buffer_s[0];
				std::cout << "Code : " << std::dec << code << std::endl;
				it = buffer_s.begin();
				buffer_s.erase(it);
				std::shared_ptr<google::protobuf::Message> msg = m_msgCatalog.deserialize(code, buffer_s);

				std::shared_ptr<Activity> msgActivity = std::dynamic_pointer_cast<Activity>(msg);
				std::cout << msgActivity->name() << std::endl;

				//m_msgDispatcher.go(msg);
			}
		}
	}
	m_bufferRecv.resize(64, 0);
	m_socket.async_receive_from(boost::asio::buffer(m_bufferRecv), m_remoteEndpoint,
		boost::bind(&UdpPeer::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpPeer::handle_send(std::vector<unsigned char>* /*message*/,
	const boost::system::error_code& /*error*/,
        std::size_t size/*bytes_transferred*/)
  	{
		std::cout << "Taille : " << std::dec << size << std::endl;
  	}

void UdpPeer::registerMessage()
{
}
