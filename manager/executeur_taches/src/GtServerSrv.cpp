#include "GtServerSrv.h"

GtServerSrv::GtServerSrv() {
  ros::NodeHandle n;
  n.param<int>("robotNumber",nb_robot,0);
  n.param<int>("teamColor",t_color,CYAN); 

  m_ei = new ExploInfoSubscriber();
}
GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id){
  m_id = id;
}

manager_msg::activity GtServerSrv::getActivityMsg(){
  return m_msg;
}

manager_msg::finalApproachingAction GtServerSrv::getFinalAppAction(){
  return m_act;
}

void GtServerSrv::interpretationZone(){
  int zone = this->m_id;
  // Get bottom-left coord of zone
  x = 0;
  y = 0;
  // Right side
  if(zone>0 && zone<13) {
    x = ((zone-1)/4)*2;
    y = ((zone-1)%4)*1.5;
  }
  // Left side
  else if (zone<=24) {
    zone -=12;
    x = -((zone-1)/4)*2 - 2;
    y = ((zone-1)%4)*1.5;
  }
  else {
    ROS_ERROR("There is only 23 zones ");
  }
  x+=0.001;
  y+=0.001;
}

int GtServerSrv::teamColorOfId(int arTag){

int team_color = 0;

switch(arTag) {
              case  C_CS1_IN    :       team_color = CYAN;       name = "CS";      break;

              case  C_CS1_OUT   :       team_color = CYAN;       name = "CS";      break;

              case  C_CS2_IN    :       team_color = CYAN;       name = "CS";      break;

              case  C_CS2_OUT   :       team_color = CYAN;       name = "CS";      break;

              case  C_RS1_IN    :       team_color = CYAN;       name = "RS";      break;

              case  C_RS1_OUT   :       team_color = CYAN;       name = "RS";      break;

              case  C_RS2_IN    :       team_color = CYAN;       name = "RS";      break;

              case  C_RS2_OUT   :       team_color = CYAN;       name = "RS";      break;

              case  C_BS_IN     :       team_color = CYAN;       name = "BS";      break;

              case  C_BS_OUT    :       team_color = CYAN;       name = "BS";      break;

              case  C_DS_IN     :       team_color = CYAN;       name = "DS";      break;

              case  C_DS_OUT    :       team_color = CYAN;       name = "DS";      break;


              case  M_CS1_IN    :       team_color = MAGENTA;    name = "CS";      break;

              case  M_CS1_OUT   :       team_color = MAGENTA;    name = "CS";      break;

              case  M_CS2_IN    :       team_color = MAGENTA;    name = "CS";      break;

              case  M_CS2_OUT   :       team_color = MAGENTA;    name = "CS";      break;

              case  M_RS1_IN    :       team_color = MAGENTA;    name = "RS";      break;

              case  M_RS1_OUT   :       team_color = MAGENTA;    name = "RS";      break;

              case  M_RS2_IN    :       team_color = MAGENTA;    name = "RS";      break;

              case  M_RS2_OUT   :       team_color = MAGENTA;    name = "RS";      break;

              case  M_BS_IN     :       team_color = MAGENTA;    name = "BS";      break;

              case  M_BS_OUT    :       team_color = MAGENTA;    name = "BS";      break;

              case  M_DS_IN     :       team_color = MAGENTA;    name = "DS";      break;

              case  M_DS_OUT    :       team_color = MAGENTA;    name = "DS";      break;

              default:      team_color = -1;     break;
              }
      return team_color;

}

bool GtServerSrv::responseToGT(manager_msg::order::Request &req,manager_msg::order::Response &res){

  ROS_INFO("No problem, I received the order ");
  setId(req.id);
  if (req.number_robot == nb_robot){
      res.number_order = req.number_order;
      res.number_robot = nb_robot;
      res.id = m_id;
      MyElements m;
      switch(req.type){ // à rajouter => machine non occupée par un robotino et au départ (on ne sait pas cs1/cs2 et rs1/rs2)
          case orderRequest::TAKE_BASE:
                m.getBS().take_base(req.parameter,nb_robot,req.number_order);
                break;
          case orderRequest::PUT_CAP:
                switch(req.parameter){
                    case orderRequest::BLACK : 
                            if(m.getCS1().getBlackCap() != 0)        m.getCS1().put_cap(req.parameter,nb_robot,req.number_order,activity::CS1);
                            else if(m.getCS2().getBlackCap() != 0)   m.getCS2().put_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                    case orderRequest::GREY : 
                            if(m.getCS1().getGreyCap() != 0)         m.getCS1().put_cap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                            else if(m.getCS2().getGreyCap() != 0)    m.getCS2().put_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                }
                break;
          case orderRequest::TAKE_CAP:
               switch(req.parameter){
                    case orderRequest::BLACK : 
                            if(m.getCS1().getBlackCap() != 0)        m.getCS1().take_cap(req.parameter,nb_robot,req.number_order,activity::CS1);
                            else if(m.getCS2().getBlackCap() != 0)   m.getCS2().take_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                    case orderRequest::GREY : 
                            if(m.getCS1().getGreyCap() != 0)         m.getCS1().take_cap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                            else if(m.getCS2().getGreyCap() != 0)    m.getCS2().take_cap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                }
                break;
          case orderRequest::PUT_RING:
                switch(req.parameter){
                    case orderRequest::GREEN :
                            if(m.getRS1().getGreenRing() != 0)       m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::YELLOW : 
                            if(m.getRS1().getYellowRing() != 0)      m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::BLUE :
                            if(m.getRS1().getBlueRing() != 0)        m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::ORANGE : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getRS1().put_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getRS2().put_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case orderRequest::TAKE_RING:
                switch(req.parameter){
                    case orderRequest::GREEN :
                            if(m.getRS1().getGreenRing() != 0)       m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::YELLOW : 
                            if(m.getRS1().getYellowRing() != 0)      m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::BLUE :
                            if(m.getRS1().getBlueRing() != 0)        m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::ORANGE : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getRS1().take_ring(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getRS2().take_ring(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case orderRequest::BRING_BASE_RS:
                switch(req.parameter){

                    case orderRequest::GREEN :
                            if(m.getRS1().getGreenRing() != 0)       m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getGreenRing() != 0)  m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::YELLOW : 
                            if(m.getRS1().getYellowRing() != 0)      m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getYellowRing() != 0) m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::BLUE :
                            if(m.getRS1().getBlueRing() != 0)        m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getBlueRing() != 0)   m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                    case orderRequest::ORANGE : 
                            if(m.getRS1().getOrangeRing() != 0)      m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS1);
                            else if(m.getRS2().getOrangeRing() != 0) m.getBS().bring_base_rs(req.parameter,nb_robot,req.number_order,activity::RS2);
                            break;
                }
                break;
          case orderRequest::DELIVER:
                switch(req.parameter){
                    case orderRequest::DS : 
                            m.getDS().deliverToDS(nb_robot,req.number_order);
                            break;
                    case orderRequest::STOCK :
                            int i = 0; 
                            for(i = 0; i<3; i++){
                                if(m.getCS1().getStockage(i) ==0 ){
                                    m.getCS1().stock(i,nb_robot,req.number_order,activity::CS1);
                                    m.getCS1().majStockID(i,1);
                                    break;
                                }
                                else if(m.getCS2().getStockage(i+3) ==0 ){
                                    m.getCS2().stock(i+3,nb_robot,req.number_order,activity::CS1);
                                    m.getCS2().majStockID(i+3,1);
                                    break;
                                }
                                else{
                                    if(i == 3 ) ROS_INFO(" ERROR : no more place to stock ");
                                } 

                            }
                }
                break;
          case orderRequest::UNCAP:
                switch(req.parameter){ // à verifier? chaque CS à des capscat spécifiques
                    case orderRequest::BLACK : 
                            if(m.getCS1().getBlackCap() != 0)        m.getCS1().uncap(req.parameter,nb_robot,req.number_order,activity::CS1);  
                            else if(m.getCS2().getBlackCap() != 0)   m.getCS2().uncap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                    case orderRequest::GREY : 
                            if(m.getCS1().getGreyCap() != 0)         m.getCS1().uncap(req.parameter,nb_robot,req.number_order,activity::CS1); 
                            else if(m.getCS2().getGreyCap() != 0)    m.getCS2().uncap(req.parameter,nb_robot,req.number_order,activity::CS2);
                            break;
                }
                break;
          case orderRequest::DESTOCK:
                if(req.id >= 0 && req.id < 3){  
                   m.getCS1().destock(req.id,nb_robot,req.number_order,activity::CS1);
                   m.getCS1().majStockID(req.id, 0);
                }
                else if(req.id >= 3 && req.id < 6){
                   m.getCS2().destock(req.id,nb_robot,req.number_order,activity::CS2);
                   m.getCS2().majStockID(req.id, 0);
                }   
                else {
                  ROS_ERROR("ERROR : req.id is not between 0 and 5 ");
                  res.accepted =false;
                }
                break;
          case orderRequest::DISCOVER:  
                interpretationZone();
                //goto               
                int16_t m_id;
                ArTagClienSrv atg;
                ROS_INFO("debugg : BEFORE ASK FOR ID");
                m_id = atg.askForId();
                int team_color = teamColorOfId(m_id);
                if(team_color != this->t_color){
                      ROS_ERROR(" Machine isn't for my team ");
                      res.accepted = false;
                      break;
                } 

                /* phase d'exploration */
                 ROS_INFO(" Starting exploring the ARTag ");
                 ReportingMachineSrvClient rm_c;
                 switch(m_id){
                      case M_BS_IN  : 
                      case M_BS_OUT :
                      case C_BS_IN  :
                      case C_BS_OUT :

                              if(m_id == C_BS_OUT || m_id == M_BS_OUT){
                                  m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                  if(m_ei->m_signals.size() != 0) {
                                          m.getBS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                  }
                              } 
                              else if (m_id == C_BS_IN || m_id == M_BS_IN){
                                  m.getBS().goTo(m.getBS().getExitMachine());
                                  m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                  if(m_ei->m_signals.size() != 0) {
                                          m.getBS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                  }
                              }
                              break;

                      case M_RS1_OUT : 
                      case M_RS1_IN  :
                      case C_RS1_IN  :
                      case C_RS1_OUT :
                      case M_RS2_OUT : 
                      case M_RS2_IN  :
                      case C_RS2_IN  :
                      case C_RS2_OUT :

                                  if(m_id == C_RS1_OUT || m_id == M_RS1_OUT){
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getRS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                      }
                                  }  
                                  else if(m_id == C_RS2_OUT || m_id == M_RS2_OUT){
                                      m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getRS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                      }
                                  }  
                                  else if(m_id == C_RS1_IN || m_id == M_RS1_IN){
                                       m.getRS1().goTo(m.getRS1().getExitMachine());
                                       m.getRS1().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getRS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                       }
                                  }  
                                  else if(m_id == C_RS2_IN || m_id == M_RS2_IN){
                                       m.getRS2().goTo(m.getRS2().getExitMachine());
                                       m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getRS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                       }
                                  }  
                              break;
                      case M_CS1_OUT : 
                      case M_CS1_IN  :
                      case C_CS1_IN  :
                      case C_CS1_OUT :
                      case M_CS2_OUT : 
                      case M_CS2_IN  :
                      case C_CS2_IN  :
                      case C_CS2_OUT : 
                              
                                  if(m_id == C_CS1_OUT || m_id == M_CS1_OUT){
                                      m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getCS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                      }
                                  }  
                                  else if(m_id == C_CS2_OUT || m_id == M_CS2_OUT){
                                      m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getCS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                      }
                                  }  

                                  else if(m_id == C_CS1_IN || m_id == M_CS1_IN){
                                       m.getCS1().goTo(m.getCS1().getExitMachine());
                                       m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getCS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                        }
                                       
                                  }  
                                  else if(m_id == C_CS2_IN || m_id == M_CS2_IN){
                                       m.getCS2().goTo(m.getCS2().getExitMachine());
                                       m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getCS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                       }
                                  }   
                              break;
                      case M_DS_IN  : 
                      case M_DS_OUT :
                      case C_DS_IN  :
                      case C_DS_OUT :
                              if(m_id == C_DS_OUT || m_id == M_DS_OUT){
                                  m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                   if(m_ei->m_signals.size() != 0) {
                                          m.getDS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                   }
                              } 
                              else if (m_id == C_DS_IN || m_id == M_DS_IN){
                                  m.getDS().goTo(m.getDS().getExitMachine());
                                  m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::FIRE);
                                  if(m_ei->m_signals.size() != 0) {
                                          m.getDS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,m_id);
                                  }
                              }
                              break;

                 }

                break;
      }

      //if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
      //else ROS_INFO(" NON DESTOCKAGE ");
      res.accepted = true;

  }
  else res.accepted = false;

  /* VERIFICATIONS */
  ROS_INFO("request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d", (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
  ROS_INFO("sending back response: nb_order=[%d], nb_robot=[%d]", (int)res.number_order, (int)res.number_robot);

  return true;
}