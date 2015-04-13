#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "UdpPeer.h"

using boost::asio::ip::udp;

/* Sert à envoyer le paquet */
void UdpPeer::send(std::shared_ptr<google::protobuf::Message>& msg){
	std::string m_buffer;		
	//std::string code = m_msgCatalog.serialize(m_buffer, msg);
	m_buffer.insert(0, "code");
	std::string m_buffer_s;
	std::string IV;
	//m_encryptUtil.encrypt(m_buffer, m_buffer_s, IV);
	m_buffer_s.insert(0, "1");
	std::string code = "code";
	m_buffer_s.insert(0, code);	
}

void UdpPeer::setCrypto(std::vector<unsigned char> key, EncryptUtils::CIPHER_TYPE cipher){
	//m_encryptUtil.setConfig(key, cipher);
}

void UdpPeer::handle_receive(const boost::system::error_code &error, std::size_t size){
	if (!error || error == boost::asio::error::message_size){
		if (m_buffer[0] != 1 /* code */) return;
		else {
			if (m_buffer[1] == '0'){ /* Le message n'est pas crypté */
				m_buffer.erase(0, 18);
				std::string code = m_buffer.substr(0, 1);
				m_buffer.erase(0, 1);
				//google::protobuf::Message* msg = m_msgCatalog.deserialize(code, m_buffer);
				//m_msgDispatcher.go(msg);
			}
			else{ /* m_buffer == '1' */
				std::string IV = m_buffer.substr(2, 16);
				m_buffer.erase(0, 18);
				std::string m_buffer_s;
				//m_encryptUtil.decrypt(m_buffer, m_buffer_s, IV);			
				std::string code = m_buffer_s.substr(0, 1);
				m_buffer_s.erase(0, 1);
				//google::protobuf::Message* msg = m_msgCatalog.deserialize(code, m_buffer_s);
				//m_msgDispatcher.go(msg);
			}

		}
	}
}

void UdpPeer::registerMessage(){

}
