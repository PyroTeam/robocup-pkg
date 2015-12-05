
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Activity.pb.h"
#include "Beacon.pb.h"
#include "UdpPeer.h"
#include "encryptUtils.h"
#include "topicToUdpEntry.h"

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
    std::string skey = "azertyuiop";
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
    //fin test EncryptUtils

    //test TopicToUdpEntry
    boost::asio::io_service io_service;
    int port;
    std::shared_ptr<UdpPeer> udpPeer(new UdpPeer(io_service, port));
    TopicToUdpEntry<comm_msg::activity> test_inpt(udpPeer, "manager/activity");


    ros::Rate loop_rate(100);
    while(ros::ok())
    {



        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
