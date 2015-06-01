#include <functional>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "udpPeer.h"
#include "encryptUtils.h"
#include "topicToUdpEntry.h"
#include "udpToTopicEntry.h"
#include "messageCatalog.h"

#include "comm_msg/activity.h"

#include <initializer_list>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "InterRobotComm");
    ros::NodeHandle nh;

    //chargement de la configuration
    int robotNumber;
    nh.param<int>("robotNumber", robotNumber, 0);

    //test EncryptUils
    /*std::string skey = "azertyuiop";
    std::vector<unsigned char> key(skey.begin(), skey.end());
    std::vector<unsigned char> iv;

    EncryptUtils eu(key, EncryptUtils::AES_CBC_128);

    std::string msg = "PyroTeam, test Encrypt Utils, robot Comm node";
    std::vector<unsigned char> message(msg.begin(), msg.end());
    std::vector<unsigned char> EncryptedMessage;
    std::vector<unsigned char> DecryptedMessage;

    eu.encrypt(message,  EncryptedMessage, iv);

    std::cout << "IV : ";
    for(auto &i: iv)
    {
        std::cout << " " << std::hex << std::uppercase << int(i);
    }
    std::cout << std::endl;
    std::cout << "Encrypted Message : ";
    for(auto &i: EncryptedMessage)
    {
        std::cout << " " << std::hex << std::uppercase << int(i);
    }
    std::cout << std::endl;

    eu.decrypt(EncryptedMessage, DecryptedMessage, iv);

    std::cout << "Decrypted Message : ";
    for(auto &i: DecryptedMessage)
    {
        std::cout << i;
    }
    std::cout << std::endl;
    //fin test EncryptUtils*/

    //test TopicToUdpEntry
    boost::asio::io_service io_service;
    int portIn = 5001;
    int portOut = 5001;
    nh.param<int>("portIn", portIn, 5001);
    nh.param<int>("portOut", portOut, 5001);
	//UdpPeer udp(io_service, port, port);
    std::shared_ptr<UdpPeer> udpPeer(new UdpPeer(io_service, portOut, portIn));

    UdpToTopicEntry<Activity, comm_msg::activity> testUdpToTopic(udpPeer, "activity");

    std::shared_ptr<MessageDispatcher> msgDispatcher(new(MessageDispatcher));
    msgDispatcher->Add<Activity>(std::function<void(google::protobuf::Message&)>(boost::bind(&UdpToTopicEntry<Activity, comm_msg::activity>::execute, &testUdpToTopic, _1)));
    udpPeer->setDispatcher(msgDispatcher);

    std::shared_ptr<MessageCatalog> msgCatalog(new(MessageCatalog));;
    msgCatalog->add<Activity>();

    udpPeer->setCatalog(msgCatalog);

    TopicToUdpEntry<comm_msg::activity> test_inpt(udpPeer, "/activity");

    //test udpToTopicEntry
	//test messageCatalog
	/*MessageCatalog msgCtg;
	msgCtg.add<Activity>();
	std::shared_ptr<Activity> activity(new Activity());
	std::shared_ptr<google::protobuf::Message> testMsg;
	std::vector<unsigned char> buffer;

	activity->set_code(1);
	activity->set_name("Status");
	activity->set_nb_robot(1);
	activity->set_state(Activity_STATE_ROBOT_END);
	activity->set_machine_used(Activity_MACHINE_TYPE_BS);
	activity->set_nb_order(3);

	testMsg = activity;

	int code = msgCtg.serialize(buffer, testMsg);
	std::cout << "Code : " << code << std::endl;

	for (auto &i : buffer)
	{
		std::cout << std::hex << std::uppercase << int(buffer[i]) << " ";
	}
	std::cout << std::endl;

	std::cout << "Debut test deserialize" << std::endl;
	std::shared_ptr<google::protobuf::Message> googleMsg = msgCtg.deserialize(1, buffer);

	std::shared_ptr<Activity> msgActivity = std::dynamic_pointer_cast<Activity>(googleMsg);
	std::cout << msgActivity->name() << std::endl;

	std::cout << "Fin test deserialize" << std::endl;*/

	std::shared_ptr<google::protobuf::Message> msgTest;

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
		//udp.send(msgTest);
		//std::cout << ".";
		//fflush(stdout);
		io_service.poll();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
