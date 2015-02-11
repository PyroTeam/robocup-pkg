/**
 * \file 		refBoxTransport.cpp
 *
 * \brief		bibliothèque de gestion de la communication avec la referee Box
 * 				pour la competition robocup (Logistic League)
 * 				Version utilisant des ports spécifiques par équipe et le
 * 				chiffrage optionel des données
 *              (Sur la base du code llsf-fake-robot.cpp)
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2014-06-16
 * \copyright	RBQT, Polytech-Lille
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

//using namespace protobuf_comm;
using namespace llsf_msgs;
using namespace fawkes;

static bool quit = false;

static boost::asio::deadline_timer *m_timer_;

refBoxTransport::refBoxTransport()
{
	m_timer_ = NULL;
	m_seq_ = 0;
	m_peer_public_ = NULL;
	m_peer_team_ = NULL;
	m_name_ = "R1";
	m_team_name_ = "RBQT";
	m_robot_number_ = 1;
	m_team_color_ = CYAN;
	m_crypto_setup_ = false;

	m_quit = false;

	m_crypto_key = "randomkey";
	m_cipher = "aes-128-cbc";

	m_config_ = NULL;
	
	m_gamePhase = PHASE_PRE_GAME;
	
}


void get_actualTime(unsigned long &sec, unsigned long &nsec)
{
    struct timespec actualTime;
    
    clock_gettime(CLOCK_REALTIME, &actualTime);
    
    sec = actualTime.tv_sec;
    nsec = actualTime.tv_nsec;
}


refBoxTransport::~refBoxTransport()
{
	delete m_timer_;
	delete m_peer_team_;
	delete m_peer_public_;
	delete m_config_;


	// Delete all global objects allocated by libprotobuf
	google::protobuf::ShutdownProtobufLibrary();
}

void refBoxTransport::init()
{
	m_config_ = new llsfrb::YamlConfiguration(CONFDIR);
	m_config_->load("config.yaml");

    //chargement des info du robot
    if (m_config_->exists("/llsfrb/game/team"))
    {
        m_team_name_ = m_config_->get_string("/llsfrb/game/team");
    }
    if (m_config_->exists("/llsfrb/game/robot_number"))
    {
        m_robot_number_ = m_config_->get_uint("/llsfrb/game/robot_number");
    }
    if (m_config_->exists("/llsfrb/game/robot_name"))
    {
        m_name_ = m_config_->get_string("/llsfrb/game/robot_name");
    }
    if (m_config_->exists("/llsfrb/game/color"))
    {
        std::string color = m_config_->get_string("/llsfrb/game/color");
        m_team_color_ = ((color == "cyan") ? CYAN : MAGENTA);
    }
    //chargement de la config réseau
	if (m_config_->exists("/llsfrb/comm/public-peer/send-port")
			&& m_config_->exists("/llsfrb/comm/public-peer/recv-port"))
	{
		m_peer_public_ = new ProtobufBroadcastPeer(
				m_config_->get_string("/llsfrb/comm/public-peer/host"),
				m_config_->get_uint("/llsfrb/comm/public-peer/recv-port"),
				m_config_->get_uint("/llsfrb/comm/public-peer/send-port"));
	} 
	else 
	{
		m_peer_public_ = new ProtobufBroadcastPeer(
				m_config_->get_string("/llsfrb/comm/public-peer/host"),
				m_config_->get_uint("/llsfrb/comm/public-peer/port"));
	}

	MessageRegister & message_register = m_peer_public_->message_register();
	message_register.add_message_type<BeaconSignal>();
	message_register.add_message_type<OrderInfo>();
	message_register.add_message_type<GameState>();
	message_register.add_message_type<VersionInfo>();
	message_register.add_message_type<ExplorationInfo>();
	message_register.add_message_type<MachineInfo>();
	message_register.add_message_type<MachineReportInfo>();
	message_register.add_message_type<RobotInfo>();


	std::string cfg_prefix = std::string("/llsfrb/comm/")
			+ ((m_team_color_ == CYAN) ? "cyan" : "magenta") + "-peer/";

	/*
	 // better to this dynamically be reacting to the public GameState
	 // this way you can also play unencrypted training games
	 std::string crypto_key = "", cipher = "aes-128-cbc";
	 try {
	 crypto_key = config_->get_string(("/llsfrb/game/crypto-keys/" + team_name_).c_str());
	 } catch (Exception &e) {
	 printf("No encryption key configured for team, not enabling crypto");
	 }
	 */

    if (m_config_->exists((cfg_prefix + "send-port").c_str())
		    && m_config_->exists((cfg_prefix + "recv-port").c_str())) 
    {
	    m_peer_team_ = new ProtobufBroadcastPeer(
			    m_config_->get_string((cfg_prefix + "host").c_str()),
			    m_config_->get_uint((cfg_prefix + "recv-port").c_str()),
			    m_config_->get_uint((cfg_prefix + "send-port").c_str()),
			    &message_register /*, crypto_key, cipher*/);
    } 
    else 
    {
	    m_peer_team_ = new ProtobufBroadcastPeer(
			    m_config_->get_string((cfg_prefix + "host").c_str()),
			    m_config_->get_uint((cfg_prefix + "port").c_str()),
			    &message_register/*, crypto_key, cipher*/);
    }


	m_peer_public_->signal_received().connect(boost::bind(&refBoxTransport::handle_message, this, _1, _2, _3, _4));
	m_peer_public_->signal_recv_error().connect(boost::bind(&refBoxTransport::handle_recv_error, this, _1, _2));
	m_peer_public_->signal_send_error().connect(boost::bind(&refBoxTransport::handle_send_error, this, _1));
	
	m_peer_team_->signal_received().connect(boost::bind(&refBoxTransport::handle_message, this, _1, _2, _3, _4));
	m_peer_team_->signal_recv_error().connect(boost::bind(&refBoxTransport::handle_recv_error, this, _1, _2));
	m_peer_team_->signal_send_error().connect(boost::bind(&refBoxTransport::handle_send_error, this, _1));

/*
#if BOOST_ASIO_VERSION >= 100601
	// Construct a signal set registered for process termination.
	boost::asio::signal_set signals(m_io_service, SIGINT, SIGTERM);

	// Start an asynchronous wait for one of the signals to occur.
	signals.async_wait(boost::bind(&refBoxTransport::signal_handler, this, _1, _2));
#endif
*/
}

void refBoxTransport::startTimer()
{
	m_timer_ = new boost::asio::deadline_timer(m_io_service);
	//m_timer_->expires_from_now(boost::posix_time::seconds(2));	
	//m_timer_->expires_from_now(boost::posix_time::time_duration(0,0,2,0));	
	m_timer_->expires_at( boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(2));

	//m_timer_->async_wait(boost::bind(&refBoxTransport::handle_timer, this, _1));
        m_timer_->wait();
	printf("Wait Timer");
}

void refBoxTransport::add_machine(std::string &name, std::string &type)
{
    m_dataMutex.lock();
    
    m_machineToReport.set_team_color(m_team_color_);
    
    llsf_msgs::MachineReportEntry *entry = m_machineToReport.add_machines();
    entry->set_name(name);
    entry->set_type(type);

    m_dataMutex.unlock();
}

void refBoxTransport::set_pose(double x, double y, double theta)
{
    m_dataMutex.lock();
 
    unsigned long sec = 0;
    unsigned long nsec = 0;
    get_actualTime(sec, nsec);
 
    Pose2D *pose = m_beaconSignal.mutable_pose();
    pose->set_x(x);
    pose->set_y(y);
    pose->set_ori(theta);

    Time *pose_time = pose->mutable_timestamp();
    pose_time->set_sec(static_cast<google::protobuf::int64>(sec));
    pose_time->set_nsec(static_cast<google::protobuf::int64>(nsec));

    m_dataMutex.unlock();
}

llsf_msgs::GameState refBoxTransport::get_gameState()
{
    llsf_msgs::GameState tempState;
    m_dataMutex.lock();
    tempState = m_gameState;
    m_dataMutex.unlock();
    return tempState;
}

llsf_msgs::ExplorationInfo refBoxTransport::get_explorationInfo()
{
    llsf_msgs::ExplorationInfo tempExpInfo;
    m_dataMutex.lock();
    tempExpInfo = m_explorationInfo;
    m_dataMutex.unlock();
    return tempExpInfo;
}


GamePhase refBoxTransport::get_phase()
{
    GamePhase tempPhase;
    m_dataMutex.lock();
    tempPhase = m_gamePhase;
    m_dataMutex.unlock();
    return tempPhase;
}


void refBoxTransport::update()
{
	m_io_service.run();
	m_io_service.reset();
}

bool refBoxTransport::isExit()
{
	return m_quit;
}

void refBoxTransport::signal_handler(const boost::system::error_code& error, int signum)
{
    if (!error) 
    {
        quit = true;

        if (m_timer_) 
        {
            m_timer_->cancel();
        }
    }
}

void refBoxTransport::handle_recv_error(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
    printf("Receive error from %s:%u: %s\n",
	 endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

void refBoxTransport::handle_send_error(std::string msg)
{
    printf("Send error: %s\n", msg.c_str());
}

void refBoxTransport::handle_message(boost::asio::ip::udp::endpoint &sender,
	       uint16_t component_id, uint16_t msg_type,
	       std::shared_ptr<google::protobuf::Message> msg)
{

    m_dataMutex.lock();

    std::shared_ptr<BeaconSignal> b;
    if ((b = std::dynamic_pointer_cast<BeaconSignal>(msg))) 
    {
#if __WORDSIZE == 64
        printf("Detected robot: %u %s:%s (seq %lu)\n",
#else
        printf("Detected robot: %u %s:%s (seq %llu)\n",
#endif
            b->number(), b->team_name().c_str(), b->peer_name().c_str(), b->seq());
    }


    std::shared_ptr<GameState> gs;
    if ((gs = std::dynamic_pointer_cast<GameState>(msg))) 
    {
        int hour = gs->game_time().sec() / 3600;
        int min  = (gs->game_time().sec() - hour * 3600) / 60;
        int sec  = gs->game_time().sec() - hour * 3600 - min * 60;

#if __WORDSIZE == 64
        printf("GameState received:  %02i:%02i:%02i.%02ld  %s %s  %u:%u points, %s vs. %s\n",
#else
        printf("GameState received:  %02i:%02i:%02i.%02lld  %s %s  %u:%u points, %s vs. %s\n",
#endif
            hour, min, sec, gs->game_time().nsec() / 1000000,
        llsf_msgs::GameState::Phase_Name(gs->phase()).c_str(),
        llsf_msgs::GameState::State_Name(gs->state()).c_str(),
        gs->points_cyan(), gs->points_magenta(),
        gs->team_cyan().c_str(), gs->team_magenta().c_str());

		if (m_team_name_ == gs->team_cyan() || m_team_name_ == gs->team_magenta()) 
		{
            if (m_team_name_ == gs->team_cyan() && m_team_color_ != CYAN) 
            {
	            printf("WARNING: sending as magenta, but our team is announced as cyan by refbox!\n");
            } 
            else if (m_team_name_ == gs->team_magenta() && m_team_color_ != MAGENTA) 
            {
	            printf("WARNING: sending as cyan, but our team is announced as magenta by refbox!\n");
            }
            if (!m_crypto_setup_) 
            {
                m_crypto_setup_ = true;

                try 
                {
                    m_crypto_key = m_config_->get_string(
                            ("/llsfrb/game/crypto-keys/" + m_team_name_).c_str());
                    printf("Set crypto key to %s (cipher %s)\n",
                    m_crypto_key.c_str(), m_cipher.c_str());
                    m_peer_team_->setup_crypto(m_crypto_key, m_cipher);
                } 
                catch (Exception &e) 
                {
                    printf("No encryption key configured for team, not enabling crypto");
                }
            }
        } 
        else if (m_crypto_setup_) 
        {
            printf("Our team is not set, training game? Disabling crypto.\n");
            m_crypto_setup_ = false;
            m_peer_team_->setup_crypto("", "");
        }
        
        m_gameState = *gs;
        
        switch (gs->phase())
        {
        default:
        case llsf_msgs::GameState::PRE_GAME:
            m_gamePhase = PHASE_PRE_GAME;
            break;
        case llsf_msgs::GameState::SETUP:
            m_gamePhase = PHASE_SETUP;
            break;
        case llsf_msgs::GameState::EXPLORATION:
            m_gamePhase = PHASE_EXPLORATION;
            break;
        case llsf_msgs::GameState::PRODUCTION:
            m_gamePhase = PHASE_PRODUCTION;
            break;
        case llsf_msgs::GameState::POST_GAME:
            m_gamePhase = PHASE_POST_GAME;
            break;
        }
	}

    std::shared_ptr<OrderInfo> oi;
    if ((oi = std::dynamic_pointer_cast<OrderInfo>(msg))) 
    {
        printf("Order Info received:\n");
        for (int i = 0; i < oi->orders_size(); ++i) 
        {
            const llsf_msgs::Order &o = oi->orders(i);
            unsigned int begin_min = o.delivery_period_begin() / 60;
            unsigned int begin_sec = o.delivery_period_begin() - begin_min * 60;
            unsigned int end_min = o.delivery_period_end() / 60;
            unsigned int end_sec = o.delivery_period_end() - end_min * 60;

            printf("  %u: %u/%u of %s from %02u:%02u to %02u:%02u at gate %s\n", o.id(),
                o.quantity_delivered(), o.quantity_requested(),
                llsf_msgs::Order::ProductType_Name(o.product()).c_str(),
                begin_min, begin_sec, end_min, end_sec,
                llsf_msgs::Order::DeliveryGate_Name(o.delivery_gate()).c_str());
        }
    }

    std::shared_ptr<VersionInfo> vi;
    if ((vi = std::dynamic_pointer_cast<VersionInfo>(msg))) 
    {
        printf("VersionInfo received: %s\n", vi->version_string().c_str());
    }

    std::shared_ptr<ExplorationInfo> ei;
    if ((ei = std::dynamic_pointer_cast<ExplorationInfo>(msg))) 
    {
        printf("ExplorationInfo received:\n");
        for (int i = 0; i < ei->signals_size(); ++i) 
        {
            const ExplorationSignal &es = ei->signals(i);
            printf("  Machine type %s assignment:", es.type().c_str());
            for (int j = 0; j < es.lights_size(); ++j) 
            {
                const LightSpec &lspec = es.lights(j);
                printf(" %s=%s", LightColor_Name(lspec.color()).c_str(),
                        LightState_Name(lspec.state()).c_str());
            }
            printf("\n");
        }
        printf("  --\n");
        for (int i = 0; i < ei->machines_size(); ++i) 
        {
            const ExplorationMachine &em = ei->machines(i);
            printf("  Machine %s at (%f, %f, %f)\n", em.name().c_str(),
                    em.pose().x(), em.pose().y(), em.pose().ori());
        }
        
        m_explorationInfo = *ei;
    }

    std::shared_ptr<MachineInfo> mi;
    if ((mi = std::dynamic_pointer_cast<MachineInfo>(msg))) 
    {
        printf("MachineInfo received:\n");
        for (int i = 0; i < mi->machines_size(); ++i) 
        {
            const Machine &m = mi->machines(i);
            const Pose2D &p = m.pose();
            printf("  %-3s|%2s|%s @ (%f, %f, %f)\n",
                    m.name().c_str(), m.type().substr(0, 2).c_str(),
            Team_Name(m.team_color()).substr(0, 2).c_str(),
                    p.x(), p.y(), p.ori());
        }
    }

    std::shared_ptr<MachineReportInfo> mri;
    if ((mri = std::dynamic_pointer_cast<MachineReportInfo>(msg))) 
    {
        printf("MachineReportInfo received:\n");
        if (mri->reported_machines_size() > 0) 
        {
            printf("  Reported machines:");
            for (int i = 0; i < mri->reported_machines_size(); ++i) 
            {
                printf(" %s", mri->reported_machines(i).c_str());
            }
            printf("\n");
        } 
        else 
        {
            printf("  no machines reported, yet\n");
        }
    }

    std::shared_ptr<RobotInfo> ri;
    if ((ri = std::dynamic_pointer_cast<RobotInfo>(msg))) 
    {
        printf("Robot Info received:\n");
        for (int i = 0; i < ri->robots_size(); ++i) 
        {
            const llsf_msgs::Robot &r = ri->robots(i);
            const llsf_msgs::Time &time = r.last_seen();
/*
            boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
            boost::posix_time::ptime last_seen =
            boost::posix_time::from_time_t(time.sec())
                + boost::posix_time::nanoseconds(time.nsec());


            boost::posix_time::time_duration const last_seen_ago_td = now - last_seen;
            float last_seen_ago = last_seen_ago_td.total_milliseconds() / 1000.f;
*/  
            float last_seen_ago = 0;
            printf("  %u %s/%s @ %s: state %s, last seen %f sec ago  Maint cyc: %u  rem: %f\n",
                    r.number(), r.name().c_str(), r.team().c_str(), r.host().c_str(),
            llsf_msgs::RobotState_Name(r.state()).substr(0,3).c_str(),
                    last_seen_ago, r.maintenance_cycles(), r.maintenance_time_remaining());
        }
    }
    
    m_dataMutex.unlock();
}


//void refBoxTransport::handle_timer(const boost::system::error_code& error)
void refBoxTransport::handle_timer()
{

    m_dataMutex.lock();
    bool error = false;
    if (! error) 
    {
        //boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
        Time *time = m_beaconSignal.mutable_time();
        //boost::posix_time::time_duration const since_epoch =
        //  now - boost::posix_time::from_time_t(0);

        //time->set_sec(static_cast<google::protobuf::int64>(since_epoch.total_seconds()));
        //time->set_nsec(
        //  static_cast<google::protobuf::int64>(since_epoch.fractional_seconds() * 
		//			   (1000000000/since_epoch.ticks_per_second())));

        unsigned long sec = 0;
        unsigned long nsec = 0;
        get_actualTime(sec, nsec);
        time->set_sec(static_cast<google::protobuf::int64>(sec));
        time->set_nsec(static_cast<google::protobuf::int64>(nsec));
    
        m_beaconSignal.set_number(m_robot_number_);
        m_beaconSignal.set_peer_name(m_name_);
        m_beaconSignal.set_team_name(m_team_name_);
        m_beaconSignal.set_team_color(m_team_color_);
        m_beaconSignal.set_seq(++m_seq_);

        m_peer_team_->send(m_beaconSignal);


        //sending machines
        if (m_machineToReport.machines_size() != 0 && m_gamePhase == PHASE_EXPLORATION)
        {
            m_machineToReport.set_team_color(m_team_color_);
            m_peer_team_->send(m_machineToReport);

        }
    
    /*
        m_timer_->expires_at(m_timer_->expires_at()
		      + boost::posix_time::milliseconds(2000));
        m_timer_->async_wait(boost::bind(&refBoxTransport::handle_timer, this, _1));
    */
    }
    m_dataMutex.unlock();
}


