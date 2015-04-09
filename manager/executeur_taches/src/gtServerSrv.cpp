#include "gtServerSrv.h"

gtServerSrv::gtServerSrv() {
  ros::NodeHandle n;
  n.param<int>("robotNumber",nb_robot,1);
}
gtServerSrv::~gtServerSrv(){}

void gtServerSrv::setId(int id){
  m_id = id;
}


bool gtServerSrv::responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res){

  if (req.number_robot == nb_robot){
      res.number_order = req.number_order;
      res.number_robot = nb_robot;
      res.id = m_id;
      if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
      else ROS_INFO(" NON DESTOCKAGE ");
      res.accepted = true;
  }
  else res.accepted = false;

  /* VERIFICATIONS */
  ROS_INFO("request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d", (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
  ROS_INFO("sending back response: nb_order=[%d], nb_robot=[%d]", (int)res.number_order, (int)res.number_robot);

  return true;
}
