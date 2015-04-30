#include "GtServerSrv.h"

GtServerSrv::GtServerSrv() {
  ros::NodeHandle n;
  n.param<int>("robotNumber",nb_robot,0);
  n.param<int>("teamColor",t_color,CYAN);

  m_msg.nb_robot = nb_robot;
  m_msg.state = manager_msg::activity::END;
  m_msg.machine_used = manager_msg::activity::NONE;

  m_ei = new ExploInfoSubscriber();
  m_ls = new LocaSubscriber();
}
GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id){
  m_id = id;
}

void GtServerSrv::going(geometry_msgs::Pose2D point){
       int count = 0, stateOfNavigation;
       do{
                   ROS_INFO("going to the point : x : %f - y : %f - theta %f",point.x,point.y,point.theta);
                   NavigationClientAction n_c;
                   stateOfNavigation = n_c.goToAPoint(point);
                   if(stateOfNavigation == deplacement_msg::MoveToPoseResult::ERROR){
                            count ++;
                            ROS_INFO("Can't go to the asked point sorry :(.. I will try another one ");
                            point.x -= 0.2;
                            point.y += 0.2;
                   }
       }while (stateOfNavigation == deplacement_msg::MoveToPoseResult::ERROR);
}
geometry_msgs::Pose2D GtServerSrv::calculOutPoint(geometry_msgs::Pose2D pt_actuel, int zone){
        geometry_msgs::Pose2D pt_dest, center;
        center.x = m_ls->machine[zone - 1].x;
        center.y = m_ls->machine[zone - 1].y;
        center.theta = m_ls->machine[zone - 1].theta;;
        pt_dest.x = 2*center.x - pt_actuel.x;
        pt_dest.y = 2*center.y - pt_actuel.y;
        pt_dest.theta = pt_actuel.theta - M_PI;
        return pt_dest;
}
void GtServerSrv::asking(geometry_msgs::Pose2D point){
      int count = 0;
      int16_t mid;
      ArTagClienSrv atg;
      FinalApproachingClient fa_c;
      do{
            if(count = 1) {point.y += 1.5; point.theta += M_PI/2;  going(point);}
            else if(count = 2) {point.x -= 2;   point.theta += M_PI/2;  going(point);}
            else if(count = 3) {point.y -= 1.5; point.theta += M_PI/2;  going(point);}
            else if(count = 4) {point.x += 2;   point.theta += M_PI/2;  going(point);}
            else count = 0;
            fa_c.starting(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
            mid = atg.askForId();
            count ++;
      }while(mid == -1);
      m_id = mid;
}
manager_msg::activity GtServerSrv::getActivityMsg(){
  return m_msg;
}

manager_msg::finalApproachingAction GtServerSrv::getFinalAppAction(){
  return m_act;
}

void GtServerSrv::interpretationZone(){
  int zone = this->m_id;
  // Get bottom-right coord of zone
  x = 0;
  y = 0;
  // Right side
  if(zone>0 && zone<13) {
    x = ((zone-1)/4)*2;
    y = ((zone-1)%4)*1.5;
    x+=2;
  }
  // Left side
  else if (zone<=24) {
    zone -=12;
    x = -((zone-1)/4)*2 - 2;
    y = ((zone-1)%4)*1.5;
    x+=2;
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

                ROS_INFO (" test  ");

                geometry_msgs::Pose2D pt_dest;
                geometry_msgs::Pose2D pt_actuel;

                if(req.id == 4) { // DS CYAN
                    pt_dest.x = 1.8;
                    pt_dest.y = 4.9;
                    pt_dest.theta = M_PI;
                    going(pt_dest);
                    pt_dest.x = 1.5;
                    pt_dest.y = 4.9;
                }
                else if (req.id == 16) { // DS MAGENTA
                    pt_dest.x = -1.8;
                    pt_dest.y = 4.9;
                    pt_dest.theta = 0;
                    going(pt_dest);
                    pt_dest.x = -1.5;
                    pt_dest.y = 4.9;
                }
                else {
                    interpretationZone();
                    pt_dest.x = this->x;
                    pt_dest.y = this->y;
                    pt_dest.theta = M_PI/4;
                }
                going(pt_dest);

                ROS_INFO ("I went to the asked point successfully ");

                ROS_INFO(" Starting exploring the ARTag ");
                asking(pt_dest);

                int team_color = teamColorOfId(m_id);
                if (team_color == CYAN)         name = "C-" + name;
                else if(team_color == MAGENTA)  name = "M-" + name;

                if(team_color != this->t_color){
                      ROS_ERROR(" Machine isn't for my team ");
                      res.accepted = false;
                      break;
                }

                /* phase d'exploration */

                 ReportingMachineSrvClient rm_c;
                 switch(m_id){
                      case M_BS_IN  :
                      case M_BS_OUT :
                      case C_BS_IN  :
                      case C_BS_OUT :

                              if(m_id == C_BS_OUT || m_id == M_BS_OUT){
                                  m_msg = m.getBS().msgToGT(nb_robot,activity::IN_PROGRESS,activity::BS,req.id);
                                  m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                  if(m_ei->m_signals.size() != 0) {
                                          m.getBS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                  }
                              }
                              else if (m_id == C_BS_IN || m_id == M_BS_IN){
                                m_msg = m.getBS().msgToGT(nb_robot,activity::IN_PROGRESS,activity::BS,req.id);
                                pt_actuel = pt_dest;
                                pt_dest = calculOutPoint(pt_actuel, req.id);
                                going(pt_dest);
                                m.getBS().startFinalAp(finalApproachingGoal::BS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                if(m_ei->m_signals.size() != 0) {
                                        m.getBS().readlights(m_ei->lSpec);
                                        m_ei->interpretationFeu();
                                        rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
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
                                      m_msg = m.getRS1().msgToGT(nb_robot,activity::IN_PROGRESS,activity::RS1,req.id);
                                      m.getBS().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getRS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"1";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                      }
                                  }
                                  else if(m_id == C_RS2_OUT || m_id == M_RS2_OUT){
                                      m_msg = m.getRS2().msgToGT(nb_robot,activity::IN_PROGRESS,activity::RS2,req.id);
                                      m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getRS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"2";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                      }
                                  }
                                  else if(m_id == C_RS1_IN || m_id == M_RS1_IN){
                                       m_msg = m.getRS1().msgToGT(nb_robot,activity::IN_PROGRESS,activity::RS1,req.id);
                                       pt_actuel = pt_dest;
                                       pt_dest = calculOutPoint(pt_actuel, req.id);
                                       going(pt_dest);
                                       m.getRS1().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getRS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"1";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                       }
                                  }
                                  else if(m_id == C_RS2_IN || m_id == M_RS2_IN){
                                       m_msg = m.getRS2().msgToGT(nb_robot,activity::IN_PROGRESS,activity::RS2,req.id);
                                       pt_actuel = pt_dest;
                                       pt_dest = calculOutPoint(pt_actuel, req.id);
                                       going(pt_dest);
                                       m.getRS2().startFinalAp(finalApproachingGoal::RS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getRS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"2";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
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
                                      m_msg = m.getCS1().msgToGT(nb_robot,activity::IN_PROGRESS,activity::CS1,req.id);
                                      m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getCS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"1";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                      }
                                  }
                                  else if(m_id == C_CS2_OUT || m_id == M_CS2_OUT){
                                      m_msg = m.getCS2().msgToGT(nb_robot,activity::IN_PROGRESS,activity::CS2,req.id);
                                      m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                      if(m_ei->m_signals.size() != 0) {
                                          m.getCS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"2";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                      }
                                  }

                                  else if(m_id == C_CS1_IN || m_id == M_CS1_IN){
                                       m_msg = m.getCS1().msgToGT(nb_robot,activity::IN_PROGRESS,activity::CS1,req.id);
                                       pt_actuel = pt_dest;
                                       pt_dest = calculOutPoint(pt_actuel, req.id);
                                       going(pt_dest);
                                       m.getCS1().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getCS1().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"1";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                        }

                                  }
                                  else if(m_id == C_CS2_IN || m_id == M_CS2_IN){
                                       m_msg = m.getCS2().msgToGT(nb_robot,activity::IN_PROGRESS,activity::CS2,req.id);
                                       pt_actuel = pt_dest;
                                       pt_dest = calculOutPoint(pt_actuel, req.id);
                                       going(pt_dest);
                                       m.getCS2().startFinalAp(finalApproachingGoal::CS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                       if(m_ei->m_signals.size() != 0) {
                                          m.getCS2().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          name = name+"2";
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                       }
                                  }
                              break;
                      case M_DS_IN  :
                      case M_DS_OUT :
                      case C_DS_IN  :
                      case C_DS_OUT :

                              if(m_id == C_DS_IN || m_id == M_DS_IN){
                                   m_msg = m.getDS().msgToGT(nb_robot,activity::IN_PROGRESS,activity::DS,req.id);
                                   m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                   if(m_ei->m_signals.size() != 0) {
                                          m.getDS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                   }
                              }
                              else if (m_id == C_DS_OUT || m_id == M_DS_OUT){
                                  m_msg = m.getDS().msgToGT(nb_robot,activity::IN_PROGRESS,activity::DS,req.id);
                                  pt_actuel = pt_dest;
                                  pt_dest = calculOutPoint(pt_actuel, req.id);
                                  going(pt_dest);
                                  //m.getDS().startFinalAp(finalApproachingGoal::DS,finalApproachingGoal::OUT,finalApproachingGoal::LIGHT);
                                  if(m_ei->m_signals.size() != 0) {
                                          m.getDS().readlights(m_ei->lSpec);
                                          m_ei->interpretationFeu();
                                          rm_c.reporting(name, m_ei->type,/*m_id*/req.id);
                                  }
                              }
                              break;

                 }

                break;
      }

      //if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
      //else ROS_INFO(" NON DESTOCKAGE ");
      m_msg = m.getBS().msgToGT(nb_robot,activity::END,activity::NONE,req.id);
      res.accepted = true;


  }
  else res.accepted = false;

  /* VERIFICATIONS */
  ROS_INFO("request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d", (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
  ROS_INFO("sending back response: nb_order=[%d], nb_robot=[%d]", (int)res.number_order, (int)res.number_robot);

  return true;
}
