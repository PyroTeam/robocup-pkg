#include "GtServerSrv.h"

#include <common_utils/zone.h>

GtServerSrv::GtServerSrv(int teamColor)
: m_nh()
, m_color(teamColor)
, m_elements(teamColor)
{
  ros::NodeHandle n;
  n.param<int>("robotNumber", m_nbrobot, 0);

  m_msg.nb_robot = m_nbrobot;
  m_msg.state = manager_msg::activity::END;
  m_msg.machine_used = manager_msg::activity::NONE;

  m_ei = new ExploInfoSubscriber();
  m_ls = new LocaSubscriber();

  m_activity_pub = n.advertise<manager_msg::activity>("manager/task_exec_state", 50);
}

GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id)
{
  m_id = id;
}

// Fonction ne servant qu'en exploration
bool GtServerSrv::going(const geometry_msgs::Pose2D &point, size_t nbAttempt)
{
  int count = 0, navState;
  geometry_msgs::Pose2D target = point;
  double xCenter = 0.0, yCenter = 0.0;

  if (common_utils::getZoneCenter(common_utils::getArea(point), xCenter, yCenter))
  {
    ROS_DEBUG("Zone center found");
  }
  else
  {
    ROS_ERROR("Unable to find zone center");
  }

  do{
    ROS_INFO("Going to point : x: %f; y: %f; theta: %f",target.x,target.y,target.theta);
    NavigationClientAction n_c;
    navState = n_c.goToAPoint(target);

    if(navState == deplacement_msg::MoveToPoseResult::ERROR)
    {
      count ++;

      double dx = xCenter - target.x;
      double dy = yCenter - target.y;

      ROS_WARN("Unable to reach requested point (%f,%f,%f). Will try another one", target.x, target.y, target.theta);

      // on décale de 30 cm la position demandée vers le centre de la machine
      target.x += 0.3*(dx/std::abs(dx));
      target.y += 0.3*(dy/std::abs(dy));
    }
  }while (navState != deplacement_msg::MoveToPoseResult::FINISHED && count <= nbAttempt);

  return (count <= nbAttempt);
}

// Trouve les points de chaque côté de la machine de la zone donnée
void GtServerSrv::getSidePoints(int zone, geometry_msgs::Pose2D &point1, geometry_msgs::Pose2D &point2)
{
  // si l'angle est OK, le premier point renvoyé est la sortie
  #define MARGIN_FROM_CENTER 0.75

  geometry_msgs::Pose2D knownMachinePose;

  knownMachinePose = m_ls->machines()[zone - 1].pose;

  geometry_msgs::Pose2D input, output;
  input.x      = 0.0;
  input.y      = -MARGIN_FROM_CENTER;
  input.theta  = M_PI_2;
  output.x     = 0.0;
  output.y     = MARGIN_FROM_CENTER;
  output.theta = -M_PI_2;

  if (m_ls->machines()[zone - 1].orientationOk)
  {
    ROS_INFO("la machine en zone %d est censee avoir le bon angle !", m_ls->machines()[zone - 1].zone);
  }
  else
  {
    ROS_INFO("je ne sais pas si la machine en zone %d a le bon angle", m_ls->machines()[zone - 1].zone);
  }

  point1 = geometry_utils::machineToMapFrame(output, knownMachinePose);
  point2 = geometry_utils::machineToMapFrame(input, knownMachinePose);

  #undef MARGIN_FROM_CENTER
}

bool GtServerSrv::knownMachineInZone(int zone)
{
  return m_ls->machines()[zone - 1].isHere;
}


manager_msg::activity GtServerSrv::getActivityMsg()
{
  return m_msg;
}

final_approach_msg::FinalApproachingAction GtServerSrv::getFinalAppAction()
{
  return m_act;
}

void GtServerSrv::interpretationZone(int zone, zoneCorner_t zoneCorner)
{
  #define ZONE_WIDTH	2.0
  #define ZONE_HEIGHT	1.5
  const float offset = 0.11;

  float xOffset = ZONE_WIDTH/2-offset;
  float yOffset = ZONE_HEIGHT/2-offset;

  if (!common_utils::getZoneCenter(zone, m_explo_target.x, m_explo_target.y))
  {
    return;
  }

  // Get corner
  switch(zoneCorner)
  {
    case BOTTOM_LEFT:
    {
      yOffset *= -1;
      xOffset *= -1;
      m_explo_target.theta = M_PI/4;
    }
    break;

    case BOTTOM_RIGHT:
    {
      yOffset *= -1;
      m_explo_target.theta = 3*M_PI/4;
    }
    break;

    case TOP_LEFT:
    {
      xOffset *= -1;
      m_explo_target.theta = -M_PI/4;
    }
    break;

    case TOP_RIGHT:
    {
      m_explo_target.theta = -3*M_PI/4;
    }
    break;

    default:
    {
      ROS_ERROR("Invalid zone corner");
    }
    break;
  }

  m_explo_target.x += xOffset;
  m_explo_target.y += yOffset;

  #undef ZONE_WIDTH
  #undef ZONE_WIDTH
}

bool GtServerSrv::isInput(int arTag)
{
  // Les INPUT sont toujours impairs
  return arTag%2 == 1;
}

void GtServerSrv::msgToGT(int stateOfOrder, int machine, int n_order) // A Verifier
{
  m_msg.nb_robot = m_nbrobot;
  m_msg.state = stateOfOrder;
  m_msg.machine_used = machine;
  m_msg.nb_order = n_order;
}

bool GtServerSrv::responseToGT(manager_msg::order::Request &req,
  manager_msg::order::Response &res)
  {
    ROS_INFO("Order received");
    ROS_INFO("Request: nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d"
    , (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);

    setId(req.id);

    if (req.number_robot == m_nbrobot)
    {
      res.number_order = req.number_order;
      res.number_robot = req.number_robot;
      res.id           = req.id;

      switch(req.type)   // à rajouter => machine non occupée par un robotino et au départ (on ne sait pas cs1/cs2 et rs1/rs2)
      {
        case orderRequest::TAKE_BASE:
        {
          // take base est forcément sur une BS si req.parameter est BLACK ou SILVER
          if (req.parameter == orderRequest::BLACK ||
            req.parameter == orderRequest::SILVER)
            {
              msgToGT(activity::IN_PROGRESS, activity::BS, req.number_order);
              ROS_INFO("Taking a Base of color : %d", req.parameter);
              //TODO: vérifier si BS disponible
              m_elements.getBS().take(req.parameter);
              msgToGT(activity::END, activity::BS, req.number_order);
            }
          }
          break;

          case orderRequest::PUT_CAP:
          {
            // déterminer de quelle machine il s'agit
            CapStation cs = m_elements.getCS(req.parameter);
            int machineName = cs.getActiviType();

            msgToGT(activity::IN_PROGRESS,machineName,req.number_order);
            cs.put_cap(req.parameter);
            msgToGT(activity::END,machineName,req.number_order);
          }
          break;

          case orderRequest::TAKE_CAP:
          {
            // déterminer de quelle machine il s'agit
            CapStation cs = m_elements.getCS(req.parameter);
            int machineName = cs.getActiviType();

            msgToGT(activity::IN_PROGRESS,machineName,req.number_order);
            cs.take_cap();
            msgToGT(activity::END,machineName,req.number_order);
          }
          break;

          case orderRequest::PUT_RING: // ne pas oublier d'avoir mis les bases avant !!
          {
            // déterminer de quelle machine il s'agit
            RingStation rs = m_elements.getRS(req.parameter);
            int machineName = rs.getActiviType();

            msgToGT(activity::IN_PROGRESS,machineName,req.number_order);
            rs.put_ring(req.parameter);
            msgToGT(activity::END,machineName,req.number_order);
          }
          break;

          case orderRequest::TAKE_RING:
          {
            // déterminer de quelle machine il s'agit
            RingStation rs = m_elements.getRS(req.parameter);
            int machineName = rs.getActiviType();

            msgToGT(activity::IN_PROGRESS,machineName,req.number_order);
            rs.take_ring();
            msgToGT(activity::END,machineName,req.number_order);
          }
          break;

          case orderRequest::BRING_BASE_RS:
          {
            /*
            Si base présente en sortie de CS
            {
            msgToGT(activity::IN_PROGRESS, activity::CS, req.number_order);
            m_elements.getCS(justUncap).takeRaw()
          }
          Sinon
          {
          msgToGT(activity::IN_PROGRESS, activity::BS, req.number_order);
          m_elements.getBS().take(req.parameter);
        }

        msgToGT(activity::IN_PROGRESS, activity::RS1, req.number_order);
        OU  msgToGT(activity::IN_PROGRESS, activity::RS2, req.number_order);

        m_elements.getRS(req.parameter).bring_base();
        msgToGT(activity::END, activity::BS, req.number_order);
        */
      }
      break;

      case orderRequest::DELIVER:
      {
        // proposition : m_elements.deliver(req.parameter); + msgToGT en dehors de cette fonction
        // TODO: fonction qui retourne la DS ou la CS la plus proche du robot où il y a un emplacement de stockage libre
        switch(req.parameter)
        {
          case orderRequest::DS :
          {
            msgToGT(activity::IN_PROGRESS,activity::DS,req.number_order);
            m_elements.getDS().deliver();
            msgToGT(activity::END,activity::DS,req.number_order);
          }
          break;

          case orderRequest::STOCK :
          // TODO: Trouver une meilleure façon de stocker / destocker
          // On ne sait pas ce qui est stocké et à quel endroit ?
          int i = 0;
          for(i = 0; i<3; i++)
          {
            if(m_elements.getCS1().getStockage(i) == 0)
            {
              m_elements.getCS1().stock(i);
              break;
            }
            else if(m_elements.getCS2().getStockage(i+3) == 0)
            {
              m_elements.getCS2().stock(i+3);
              break;
            }
            else
            {
              if(i == 3 ) ROS_ERROR("ERROR: no more place to stock ");
            }
          }
          break;
        }
      }
      break;

      case orderRequest::UNCAP:
      // Après un UNCAP, la base est amenée en sortie de la CS et ne sert que comme ADDITIONNAL BASE
      // Peut-être faudrait il prévoir d'amener cette base à une RS ?
      {
        // déterminer de quelle machine il s'agit
        CapStation cs = m_elements.getCS(req.parameter);
        int machineName = cs.getActiviType();

        msgToGT(activity::IN_PROGRESS,machineName,req.number_order);
        cs.uncap();
        msgToGT(activity::END,machineName,req.number_order);
      }
      break;

      case orderRequest::DESTOCK:
      {
        // TODO: Trouver une meilleure façon de stocker / destocker
        if(req.id >= 0 && req.id < 3)
        {
          msgToGT(activity::IN_PROGRESS,activity::CS1,req.number_order);
          m_elements.getCS1().destock(req.id);
          res.accepted = true;
          res.needToResendOrder = false;
          msgToGT(activity::END,activity::CS1,req.number_order);
        }
        else if(req.id >= 3 && req.id < 6)
        {
          msgToGT(activity::IN_PROGRESS,activity::CS2,req.number_order);
          m_elements.getCS2().destock(req.id);
          res.accepted = true;
          res.needToResendOrder = false;
          msgToGT(activity::END,activity::CS2,req.number_order);
        }
        else
        {
          ROS_ERROR("ERROR: req.id is not between 0 and 5 ");
          res.accepted =false;
          res.needToResendOrder = true;
        }
      }
      break;

      case orderRequest::DISCOVER:
      {
        Machine *machine = NULL;
        geometry_msgs::Pose2D firstSidePoint, secondSidePoint;
        geometry_msgs::Pose2D tmpFirstPoint, tmpSecondPoint;

        int machineSideId = 0;
        ReportingMachineSrvClient reportClient;

        ROS_INFO("Let's explore zone %d", req.id);

        m_ls->spin();

        res.accepted = true;
        res.needToResendOrder =  false;

        // si on ne connait pas la machine à cet instant et qu'on ne connait pas toutes les machines
        if (!knownMachineInZone(req.id) && !m_ls->haveAllTheMachines())
        {
          // A partir de zone -> déterminer premier coin zone (plus accessible)
          // TODO: choix judicieux du coin à déterminer
          interpretationZone(req.id, BOTTOM_LEFT);

          //ROS_INFO("Point Target Bottom Left (%f, %f) theta: %f", m_explo_target.x, m_explo_target.y, m_explo_target.theta);
          ROS_INFO("Point Target Bottom Left");

          // Se déplacer au premier coin zone
          // on redemande au maximume 3 fois en cas de collision avec un mur
          if (!going(m_explo_target, 3))
          {
            res.accepted = false;
            res.needToResendOrder = true;
            break;
          }
          // refresh machines
          m_ls->spin();

          // Si machine NON présente
          if (!knownMachineInZone(req.id) && !m_ls->haveAllTheMachines())
          {
            ROS_INFO("No known Machine in this area %d", req.id);
            // TODO: choix judicieux du coin à déterminer
            interpretationZone(req.id, BOTTOM_RIGHT);
            //ROS_INFO("Point Target BottRight (%f, %f) theta: %f", m_explo_target.x, m_explo_target.y, m_explo_target.theta);
            ROS_INFO("Point Target Bottom Right");

            // Se rendre au second coin zone
            if (!going(m_explo_target, 3))
            {
              res.accepted = false;
              res.needToResendOrder = true;
              break;
            }
            // refresh machines
            m_ls->spin();

            // A partir detection machine -> voir si machine présente
            // Si machine toujours NON présente, abandon
            if (!knownMachineInZone(req.id))
            {
              // TODO: abandonner le service
              ROS_INFO("There is definitely no machine in this zone %d. Abort request", req.id);
              res.accepted = false;
              res.needToResendOrder = false;
              break;
            }
          }
          else if (!knownMachineInZone(req.id) && m_ls->haveAllTheMachines())
          {
            // TODO: abandonner le service
            ROS_INFO("There is definitely no machine in this zone %d. Abort request", req.id);
            res.accepted = false;
            res.needToResendOrder = false;
            break;
          }
        }
        else if (!knownMachineInZone(req.id) && m_ls->haveAllTheMachines())
        {
          // TODO: abandonner le service
          ROS_INFO("There is definitely no machine in this zone %d. Abort request", req.id);
          res.accepted = false;
          res.needToResendOrder = false;
          break;
        }

        // Si machine présente, déterminer point devant machine

        // Calculer les deux points devant la machine
        m_ls->spin();
        getSidePoints(req.id, tmpFirstPoint, tmpSecondPoint);
        firstSidePoint = tmpFirstPoint;
        secondSidePoint = tmpSecondPoint;

        // Se rendre au point devant la machine
        // utiliser le point le plus proche

        // Si l'orientation de la machine est bonne, on choisit le premier point,
        // qui est la sortie de la machine (sauf pour la DS)
        if (m_ls->machines()[req.id-1].orientationOk)
        {
          if (!machine->isDS(machineSideId))
          {
            firstSidePoint = tmpFirstPoint;
            secondSidePoint = tmpSecondPoint;
          }
          else
          {
            firstSidePoint = tmpSecondPoint;
            secondSidePoint = tmpFirstPoint;
          }
        }
        // Sinon on se dirige vers le point le plus proche
        else
        {
          geometry_msgs::Pose2D actualPose = m_poseSub.getPose2D();
          double firstDistance = geometry_utils::distance(actualPose,tmpFirstPoint);
          double secondDistance = geometry_utils::distance(actualPose,tmpSecondPoint);

          if(secondDistance < firstDistance)
          {
            firstSidePoint = tmpSecondPoint;
            secondSidePoint = tmpFirstPoint;
          }
        }

        // TODO: Le going ci-dessous peut avoir demandé un déplacement très
        // long et qui plus est sur une machine que l'on n'avait jamais
        // réellement vue (mirroring de machines).
        // Il est donc possible et probable qu'on ne soit pas face à la machine
        // Il est nécéssaire de refaire la procédure de going dans ce cas
        // TODO: A decomenter pour tester et / ou integrer
        bool use_workaround = false;
        m_nh.getParamCached("/workaround", use_workaround);

        // TODO: gérer les cas d'erreurs de going
        // Ne pas abort si on utilise le workaround, il faut réessayer un nouveau side point
        if (!going(firstSidePoint) && !use_workaround)
        {
          res.accepted = false;
          res.needToResendOrder = true;
          break;
        }

        m_ls->spin();
        if (use_workaround)
        {
          ROS_WARN_ONCE("Exec Task workaround currently in use !!!");
          const float sidePointsMargin = 0.06; // 6cm
          geometry_msgs::Pose2D oldTmpFirstPoint = tmpFirstPoint;
          getSidePoints(req.id, tmpFirstPoint, tmpSecondPoint);
          float dist = geometry_utils::distance(tmpFirstPoint, oldTmpFirstPoint);
          if (dist > sidePointsMargin)
          {
            ROS_WARN("Robot was too badly placed, maybe after a swapped exploration. (Error: %f m). Will retry once.", dist);

            firstSidePoint = tmpFirstPoint;
            secondSidePoint = tmpSecondPoint;

            // Se rendre au point devant la machine
            // utiliser le point le plus proche

            // Si l'orientation de la machine est bonne, on choisit le premier point,
            // qui est la sortie de la machine (sauf pour la DS)
            if (m_ls->machines()[req.id-1].orientationOk)
            {
              if (!machine->isDS(machineSideId))
              {
                firstSidePoint = tmpFirstPoint;
                secondSidePoint = tmpSecondPoint;
              }
              else
              {
                firstSidePoint = tmpSecondPoint;
                secondSidePoint = tmpFirstPoint;
              }
            }
            // Sinon on se dirige vers le point le plus proche
            else
            {
              geometry_msgs::Pose2D actualPose = m_poseSub.getPose2D();
              double firstDistance = geometry_utils::distance(actualPose,tmpFirstPoint);
              double secondDistance = geometry_utils::distance(actualPose,tmpSecondPoint);

              if(secondDistance < firstDistance)
              {
                firstSidePoint = tmpSecondPoint;
                secondSidePoint = tmpFirstPoint;
              }
            }

            // TODO: gérer les cas d'erreurs de going
            if (!going(firstSidePoint))
            {
              res.accepted = false;
              res.needToResendOrder = true;
              break;
            }

            m_ls->spin();

          }
        }


        // Récupérer ArTag ID
        // TODO: mettre ArTagClient en membre de classe
        ArTagClienSrv atg;
        machineSideId = atg.askForId();
        for(int i=0; i < 3 ; ++i)
        {
          if(!common_utils::exists(machineSideId))
          {
            usleep(100000);
            machineSideId = atg.askForId();
          }
        }

        machine = m_elements.getMachineFromTag(machineSideId);
        if (machine == nullptr)
        {
          ROS_ERROR("Unable to get correct AR Tag from this machine. Abort service");
          res.accepted = false;
          res.needToResendOrder = true;
          break;
        }

        machine->majCenter(m_ls->machines()[req.id - 1].pose);

        // Vérifier si INPUT (TODO: à vérifier)
        if(isInput(machineSideId) && !machine->isDS(machineSideId))
        {
          // Si OUI
          machine->majEntry(firstSidePoint);
          machine->majExit(secondSidePoint);
          ROS_INFO("I see an input of a machine with the angle %f", machine->getCenterMachine().theta);

          // Se rendre ou point devant autre côté de la machine
          if (!going(secondSidePoint))
          {
            res.accepted = false;
            res.needToResendOrder = true;
            break;
          }
          m_ls->spin();
          // Récupérer ArTag ID
          machineSideId = atg.askForId();
          for(int i=0; i < 3 ; ++i)
          {
            if(!common_utils::exists(machineSideId))
            {
              usleep(100000);
              machineSideId = atg.askForId();
            }
          }

          // Vérifier si INPUT, si OUI abandonner
          if(isInput(machineSideId))
          {
            // TODO: abandonner le service
            ROS_ERROR("Unable to reach output for this MPS. Abort service");
            res.accepted = false;
            res.needToResendOrder = true;
            break;
          }
        }
        else
        {
          machine->majEntry(secondSidePoint);
          machine->majExit(firstSidePoint);
        }


        // Approche finale, objectif FEU
        // TODO: Uncomment
        FinalApproachingClient fa_c;
        machineSideId = atg.askForId();
        for(int i=0; i < 3 ; ++i)
        {
          if(!common_utils::exists(machineSideId))
          {
            usleep(100000);
            machineSideId = atg.askForId();
          }
        }

        if(machine->isDS(machineSideId))
        {
          fa_c.starting(machine->getFaType(), FinalApproachingGoal::IN, FinalApproachingGoal::LIGHT);
        }
        else
        {
          fa_c.starting(machine->getFaType(), FinalApproachingGoal::OUT, FinalApproachingGoal::LIGHT);
        }

        if(fa_c.getSuccess())
        {
          // Traitement d'image, détection FEU
          FeuClientAction f_c;
          f_c.lightsStates(m_ei->m_lSpec);

          // Interprétation type à partir de LightSignal
          m_ei->interpretationFeu();

          // Reporter machine
          reportClient.reporting(machine->getName(), m_ei->type, req.id);
        }
        else
        {
          // Reporter machine avec feu vide
          reportClient.reporting(machine->getName(), "", req.id);
        }

        m_ls->spin();

      } // end of discover order
      break;

      default:
      break;
    }
    //if(req.id != 0) ROS_INFO(" DESTOCKAGE à l'endroit d'id = %d", (int) req.id);
    //else ROS_INFO(" NON DESTOCKAGE ");
    msgToGT(activity::END,activity::NONE,req.id);
    // res.accepted = true;
    // res.needToResendOrder =  false;
  }
  else
  {
    ROS_WARN("Request for another robot");
    res.accepted = false;
    res.needToResendOrder = true;
  }

  /* VERIFICATIONS */
  ROS_INFO("Requested (reminder): nb_order=%d, nb_robot=%d, type=%d, parameter=%d, id=%d"
  , (int)req.number_order, (int)req.number_robot, (int)req.type, (int)req.parameter, (int)req.id);
  ROS_INFO("Response: nb_order=%d, nb_robot=%d", (int)res.number_order, (int)res.number_robot);

  return true;
}
