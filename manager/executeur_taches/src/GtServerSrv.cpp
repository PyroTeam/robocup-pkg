#include "GtServerSrv.h"

GtServerSrv::GtServerSrv() {
  ros::NodeHandle n;
  n.param<int>("robotNumber",nb_robot,0); 
}
GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id){
  m_id = id;
}

manager_msg::activity GtServerSrv::getActivityMsg(){
  return m_msg;
}

bool GtServerSrv::responseToGT(manager_msg::order::Request  &req,manager_msg::order::Response &res){

  if (req.number_robot == nb_robot){
      res.number_order = req.number_order;
      res.number_robot = nb_robot;
      res.id = m_id;
      res.accepted = true;
      MyElements m;
      switch(req.type){ // à rajouter => machine non occupée par un robotino et au départ (on ne sait pas cs1/cs2 et rs1/rs2)
          case 0:
                m.getBS().take_base(req.parameter,nb_robot,req.number_order);
                break;
          case 1:
                switch(req.parameter){
                    case 10 : 
                            if(m.getCS1().getBlackCap() != 0)        m.getCS1().put_cap(req.parameter,nb_robot,req.number_order,activity::CS1);
                            else if(m.getCS2().getBlackCap() != 0)   m.getCS2().put_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                    case 17 : 
                            if(m.getCS1().getGreyCap() != 0)         m.getCS1().put_cap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                            else if(m.getCS2().getGreyCap() != 0)    m.getCS2().put_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                }
                break;
          case 2:
               switch(req.parameter){
                        case 10 : 
                                if(m.getCS1().getBlackCap() != 0)        m.getCS1().take_cap(req.parameter,nb_robot,req.number_order,activity::CS1);
                                else if(m.getCS2().getBlackCap() != 0)   m.getCS2().take_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                                break;
                        case 17 : 
                                if(m.getCS1().getGreyCap() != 0)         m.getCS1().take_cap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                                else if(m.getCS2().getGreyCap() != 0)    m.getCS2().take_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                                break;
                    }
                break;
          case 3:
                switch(req.parameter){
                    case 16 :
                            if(m.getRS1().getGreenRing() != 0)       m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 14 : 
                            if(m.getRS1().getYellowRing() != 0)      m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 15 :
                            if(m.getRS1().getBlueRing() != 0)        m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 13 : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case 4:
                switch(req.parameter){
                    case 16 :
                            if(m.getRS1().getGreenRing() != 0)       m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 14 : 
                            if(m.getRS1().getYellowRing() != 0)      m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 15 :
                            if(m.getRS1().getBlueRing() != 0)        m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 13 : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case 5:
                switch(req.parameter){

                    case 16 :
                            if(m.getRS1().getGreenRing() != 0)       m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 14 : 
                            if(m.getRS1().getYellowRing() != 0)      m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 15 :
                            if(m.getRS1().getBlueRing() != 0)        m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case 13 : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case 6:
                switch(req.parameter){
                    case 18 : 
                            m.getDS().deliverToDS(nb_robot,req.number_order);
                            break;
                    case 19 :
                            int i = 0; 
                            for(i = 0; i<3; i++){
                                if(m.getCS1().getStockage(i) ==0 ){
                                    m.getCS1().majStockID(i, 1);
                                    m.getCS1().stock(i,nb_robot,req.number_order,activity::CS1);
                                    break;
                                }
                                else if(m.getCS2().getStockage(i+3) ==0 ){
                                    m.getCS2().majStockID(i+3, 1);
                                    m.getCS2().stock(i+3,nb_robot,req.number_order,activity::CS1);
                                    break;
                                }
                                else{
                                    if(i == 3 ) ROS_INFO(" ERROR : no more place to stock ");
                                } 

                            }
                }
                break;
          case 7:
                switch(req.parameter){ // à verifier? chaque CS à des capscat spécifiques
                    case 10 : 
                            if(m.getCS1().getBlackCap() != 0)        m.getCS1().uncap(req.parameter,nb_robot,req.number_order,activity::CS1);  
                            else if(m.getCS2().getBlackCap() != 0)   m.getCS2().uncap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                    case 17 : 
                            if(m.getCS1().getGreyCap() != 0)         m.getCS1().uncap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                            else if(m.getCS2().getGreyCap() != 0)    m.getCS2().uncap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                }
                break;
          case 8:
                if(req.id >= 0 && req.id < 3){  
                   m.getCS1().destock(req.id,nb_robot,req.number_order,activity::CS1);
                   m.getCS1().majStockID(req.id, 0);
                }
                else if(req.id >= 3 && req.id < 6){
                   m.getCS2().destock(req.id,nb_robot,req.number_order,activity::CS2);
                   m.getCS2().majStockID(req.id, 0);
                }   
                else("ERROR : req.id is not between 0 and 5 ");
                break;
          case 9:
                /* EXPLORE */
                break;
      }

      if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
      else ROS_INFO(" NON DESTOCKAGE ");

  }
  else res.accepted = false;

  /* VERIFICATIONS */
  ROS_INFO("request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d", (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
  ROS_INFO("sending back response: nb_order=[%d], nb_robot=[%d]", (int)res.number_order, (int)res.number_robot);

  return true;
}