#include <ros/ros.h>
#include "manager_msg/order.h"
#include "srvorder.h"

Srvorder::Srvorder(ros::Time game_time,int nb_order,int nb_robot,int type,int parametre,int id){
        ROS_INFO("Send order");
        m_client = m_nh.serviceClient<manager_msg::order>("manager/order");
        m_srv.request.game_time = game_time;
        m_srv.request.number_order = nb_order;
        m_srv.request.number_robot = nb_robot;
        m_srv.request.type = type;
        m_srv.request.parameter = parametre;
        m_srv.request.id = id;
        m_client.call(m_srv);
        m_accepted = m_srv.response.accepted;
        m_numberOrder = m_srv.response.number_robot;
        m_numberRobot = m_srv.response.number_robot;
        m_id = m_srv.response.id;
}

Srvorder::~Srvorder(){}
