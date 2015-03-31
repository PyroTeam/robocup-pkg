/**
 * \file 		RefBoxTransport.cpp
 *
 * \brief		bibliothèque de gestion de la communication avec la referee Box
 * 				pour la competition robocup (Logistic League)
 * 				Version utilisant des ports spécifiques par équipe et le
 * 				chiffrage optionel des données
 *              (Sur la base du code llsf-fake-robot.cpp)
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2014-06-16
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

//#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include <config/yaml.h>

#include <protobuf_comm/peer.h>
#include <utils/system/argparser.h>
#include <time.h>

#include <msgs/BeaconSignal.pb.h>
#include <msgs/OrderInfo.pb.h>
#include <msgs/GameState.pb.h>
#include <msgs/VersionInfo.pb.h>
#include <msgs/ExplorationInfo.pb.h>
#include <msgs/MachineInfo.pb.h>
#include <msgs/MachineReport.pb.h>
#include <msgs/RobotInfo.pb.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/time_traits.hpp>
#include <boost/date_time.hpp>

#include "refBoxTransport.h"
#include "refBoxComm.h"

//using namespace protobuf_comm;
using namespace llsf_msgs;
using namespace fawkes;

static bool quit = false;

static boost::asio::deadline_timer *m_timer;

RefBoxTransport::RefBoxTransport()
{
	m_timer = NULL;
	m_seq = 0;
	m_peer_public = NULL;
	m_peer_team = NULL;
	
	m_crypto_setup = false;

	m_quit = false;

	m_crypto_key = "randomkey";
	m_cipher = "aes-128-cbc";

	m_config = NULL;	
}


void get_actualTime(unsigned long &sec, unsigned long &nsec)
{
    struct timespec actualTime;
    
    clock_gettime(CLOCK_REALTIME, &actualTime);
    
    sec = actualTime.tv_sec;
    nsec = actualTime.tv_nsec;
}


RefBoxTransport::~RefBoxTransport()
{
	delete m_peer_team;
	delete m_peer_public;
	delete m_config;


	// Delete all global objects allocated by libprotobuf
	google::protobuf::ShutdownProtobufLibrary();
}

void RefBoxTransport::init(std::string teamColor)
{
	m_config = new llsfrb::YamlConfiguration(CONFDIR);
	m_config->load("config.yaml");

    //chargement des info du robot
    if (m_config->exists("/llsfrb/game/team"))
    {
        m_team_name = m_config->get_string("/llsfrb/game/team");
    }
    
    m_team_color = ((teamColor == "cyan") ? CYAN : MAGENTA);
    
    //chargement de la config réseau
	if (m_config->exists("/llsfrb/comm/public-peer/send-port")
			&& m_config->exists("/llsfrb/comm/public-peer/recv-port"))
	{
		m_peer_public = new ProtobufBroadcastPeer(
				m_config->get_string("/llsfrb/comm/public-peer/host"),
				m_config->get_uint("/llsfrb/comm/public-peer/recv-port"),
				m_config->get_uint("/llsfrb/comm/public-peer/send-port"));
	} 
	else 
	{
		m_peer_public = new ProtobufBroadcastPeer(
				m_config->get_string("/llsfrb/comm/public-peer/host"),
				m_config->get_uint("/llsfrb/comm/public-peer/port"));
	}

	MessageRegister & message_register = m_peer_public->message_register();
	message_register.add_message_type<BeaconSignal>();
	message_register.add_message_type<OrderInfo>();
	message_register.add_message_type<GameState>();
	message_register.add_message_type<VersionInfo>();
	message_register.add_message_type<ExplorationInfo>();
	message_register.add_message_type<MachineInfo>();
	message_register.add_message_type<MachineReportInfo>();
	message_register.add_message_type<RobotInfo>();


	std::string cfg_prefix = std::string("/llsfrb/comm/")
			+ ((m_team_color == CYAN) ? "cyan" : "magenta") + "-peer/";

    if (m_config->exists((cfg_prefix + "send-port").c_str())
		    && m_config->exists((cfg_prefix + "recv-port").c_str())) 
    {
	    m_peer_team = new ProtobufBroadcastPeer(
			    m_config->get_string((cfg_prefix + "host").c_str()),
			    m_config->get_uint((cfg_prefix + "recv-port").c_str()),
			    m_config->get_uint((cfg_prefix + "send-port").c_str()),
			    &message_register /*, crypto_key, cipher*/);
    } 
    else 
    {
	    m_peer_team = new ProtobufBroadcastPeer(
			    m_config->get_string((cfg_prefix + "host").c_str()),
			    m_config->get_uint((cfg_prefix + "port").c_str()),
			    &message_register/*, crypto_key, cipher*/);
    }


	m_peer_public->signal_received().connect(boost::bind(&RefBoxTransport::handle_message, this, _1, _2, _3, _4));
	m_peer_public->signal_recv_error().connect(boost::bind(&RefBoxTransport::handle_recv_error, this, _1, _2));
	m_peer_public->signal_send_error().connect(boost::bind(&RefBoxTransport::handle_send_error, this, _1));
	
	m_peer_team->signal_received().connect(boost::bind(&RefBoxTransport::handle_message, this, _1, _2, _3, _4));
	m_peer_team->signal_recv_error().connect(boost::bind(&RefBoxTransport::handle_recv_error, this, _1, _2));
	m_peer_team->signal_send_error().connect(boost::bind(&RefBoxTransport::handle_send_error, this, _1));

/*
#if BOOST_ASIO_VERSION >= 100601
	// Construct a signal set registered for process termination.
	boost::asio::signal_set signals(m_io_service, SIGINT, SIGTERM);

	// Start an asynchronous wait for one of the signals to occur.
	signals.async_wait(boost::bind(&RefBoxTransport::signal_handler, this, _1, _2));
#endif
*/
}

void RefBoxTransport::startTimer()
{
	m_timer = new boost::asio::deadline_timer(m_io_service);
	//m_timer_->expires_from_now(boost::posix_time::seconds(2));	
	//m_timer_->expires_from_now(boost::posix_time::time_duration(0,0,2,0));	
	m_timer->expires_at( boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(2));

	//m_timer_->async_wait(boost::bind(&RefBoxTransport::handle_timer, this, _1));
    m_timer->wait();
	printf("Wait Timer");
}

void RefBoxTransport::update()
{
	m_io_service.run();
	m_io_service.reset();
}

bool RefBoxTransport::isExit()
{
	return m_quit;
}

void RefBoxTransport::signal_handler(const boost::system::error_code& error, int signum)
{
    if (!error) 
    {
        quit = true;

        if (m_timer) 
        {
            m_timer->cancel();
        }
    }
}

void RefBoxTransport::handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
    printf("Receive error from %s:%u: %s\n",
	 endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

void RefBoxTransport::handle_send_error(std::string msg)
{
    printf("Send error: %s\n", msg.c_str());
}

void RefBoxTransport::handle_message(boost::asio::ip::udp::endpoint &sender,
	       uint16_t component_id, uint16_t msg_type,
	       std::shared_ptr<google::protobuf::Message> msg)
{

    m_dataMutex.lock();
    
    
    //first we test if we need to change connexion to encryption mode
    std::shared_ptr<GameState> gs;
    if ((gs = std::dynamic_pointer_cast<GameState>(msg))) 
    {
		if (m_team_name == gs->team_cyan() || m_team_name == gs->team_magenta()) 
		{
            if (m_team_name == gs->team_cyan() && m_team_color != CYAN) 
            {
	            printf("WARNING: sending as magenta, but our team is announced as cyan by refbox!\n");
            } 
            else if (m_team_name == gs->team_magenta() && m_team_color != MAGENTA) 
            {
	            printf("WARNING: sending as cyan, but our team is announced as magenta by refbox!\n");
            }
            if (!m_crypto_setup) 
            {
                m_crypto_setup = true;

                try 
                {
                    m_crypto_key = m_config->get_string(
                            ("/llsfrb/game/crypto-keys/" + m_team_name).c_str());
                    //printf("Set crypto key to %s (cipher %s)\n",
                    //m_crypto_key.c_str(), m_cipher.c_str());
                    m_peer_team->setup_crypto(m_crypto_key, m_cipher);
                } 
                catch (Exception &e) 
                {
                    printf("No encryption key configured for team, not enabling crypto");
                }
            }
        } 
        else if (m_crypto_setup) 
        {
            printf("Our team is not set, training game? Disabling crypto.\n");
            m_crypto_setup = false;
            m_peer_team->setup_crypto("", "");
        }
	}

    //then we process messages
    m_msgDispatch->Go(*msg);
 
    m_dataMutex.unlock();
}

