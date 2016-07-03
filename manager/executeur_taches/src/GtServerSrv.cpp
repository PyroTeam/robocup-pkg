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
  m_rp = new RobotPoseSubscriber();

  m_activity_pub = n.advertise<manager_msg::activity>("manager/task_exec_state", 50);
}

GtServerSrv::~GtServerSrv(){}

void GtServerSrv::setId(int id)
{
  m_id = id;
}

bool GtServerSrv::machineIsDs(int id)
{
  if(id == C_DS_IN || id == C_DS_OUT || id == M_DS_IN || id == M_DS_OUT)
  {
    return true;
  }
  return false;
}

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

/* Valentin's function */
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
        ROS_WARN("la machine en zone %d est censee avoir le bon angle !", m_ls->machines()[zone - 1].zone);
    }
    else
    {
        ROS_WARN("je ne sais pas si la machine en zone %d a le bon angle", m_ls->machines()[zone - 1].zone);
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
    #define ZONE_WIDTH	1.96
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
            yOffset *= -1;
            xOffset *= -1;
            m_explo_target.theta = M_PI/4;
        break;

        case BOTTOM_RIGHT:
            yOffset *= -1;
            m_explo_target.theta = 3*M_PI/4;
        break;

        case TOP_LEFT:
            xOffset *= -1;
            m_explo_target.theta = -M_PI/4;
        break;

        case TOP_RIGHT:
            m_explo_target.theta = -3*M_PI/4;
        break;

        default:
            ROS_ERROR("Invalid zone corner");
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

bool GtServerSrv::responseToGT(manager_msg::order::Request &req,manager_msg::order::Response &res)
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
      // take base est forcément sur une BS
      // proposition : m_elements.take_base(req.parameter); + msgToGT en dehors de cette fonction
      m_elements.getBS().take_base(req.parameter,m_nbrobot,req.number_order);
      break;
      case orderRequest::PUT_CAP:
      // proposition : m_elements.put_cap(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la CS correspondant au paramètre de couleur
      switch(req.parameter)
      {
        case orderRequest::BLACK :
        if(m_elements.getCS1().getBlackCap() != 0)        m_elements.getCS1().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getBlackCap() != 0)   m_elements.getCS2().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
        case orderRequest::GREY :
        if(m_elements.getCS1().getGreyCap() != 0)         m_elements.getCS1().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getGreyCap() != 0)    m_elements.getCS2().put_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
      }
      break;
      case orderRequest::TAKE_CAP:
      // proposition : m_elements.take_cap(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la CS correspondant au paramètre de couleur
      switch(req.parameter)
      {
        case orderRequest::BLACK :
        if(m_elements.getCS1().getBlackCap() != 0)        m_elements.getCS1().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getBlackCap() != 0)   m_elements.getCS2().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
        case orderRequest::GREY :
        if(m_elements.getCS1().getGreyCap() != 0)         m_elements.getCS1().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getGreyCap() != 0)    m_elements.getCS2().take_cap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
      }
      break;
      case orderRequest::PUT_RING:
      // proposition : m_elements.put_ring(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la RS correspondant au paramètre de couleur
      switch(req.parameter)
      {
        case orderRequest::GREEN :
        if(m_elements.getRS1().getGreenRing() != 0)       m_elements.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getGreenRing() != 0)  m_elements.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::YELLOW :
        if(m_elements.getRS1().getYellowRing() != 0)      m_elements.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getYellowRing() != 0) m_elements.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::BLUE :
        if(m_elements.getRS1().getBlueRing() != 0)        m_elements.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getBlueRing() != 0)   m_elements.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::ORANGE :
        if(m_elements.getRS1().getOrangeRing() != 0)      m_elements.getRS1().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getOrangeRing() != 0) m_elements.getRS2().put_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
      }
      break;
      case orderRequest::TAKE_RING:
      // proposition : m_elements.take_ring(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la RS correspondant au paramètre de couleur
      switch(req.parameter)
      {
        case orderRequest::GREEN :
        if(m_elements.getRS1().getGreenRing() != 0)       m_elements.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getGreenRing() != 0)  m_elements.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::YELLOW :
        if(m_elements.getRS1().getYellowRing() != 0)      m_elements.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getYellowRing() != 0) m_elements.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::BLUE :
        if(m_elements.getRS1().getBlueRing() != 0)        m_elements.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getBlueRing() != 0)   m_elements.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::ORANGE :
        if(m_elements.getRS1().getOrangeRing() != 0)      m_elements.getRS1().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getOrangeRing() != 0) m_elements.getRS2().take_ring(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
      }
      break;
      case orderRequest::BRING_BASE_RS:
      // proposition : m_elements.bring_base(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la RS correspondant au paramètre de couleur
      switch(req.parameter)
      {
        case orderRequest::GREEN :
        if(m_elements.getRS1().getGreenRing() != 0)       m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getGreenRing() != 0)  m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::YELLOW :
        if(m_elements.getRS1().getYellowRing() != 0)      m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getYellowRing() != 0) m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::BLUE :
        if(m_elements.getRS1().getBlueRing() != 0)        m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getBlueRing() != 0)   m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
        case orderRequest::ORANGE :
        if(m_elements.getRS1().getOrangeRing() != 0)      m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS1);
        else if(m_elements.getRS2().getOrangeRing() != 0) m_elements.getBS().bring_base_rs(req.parameter,m_nbrobot,req.number_order,activity::RS2);
        break;
      }
      break;
      case orderRequest::DELIVER:
      // proposition : m_elements.deliver(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la DS ou la CS la plus proche du robot où il y a un emplacement de stockage libre
      switch(req.parameter)
      {
        case orderRequest::DS :
        m_elements.getDS().deliverToDS(m_nbrobot,req.number_order);
        break;
        case orderRequest::STOCK :
        // On ne sait pas ce qui est stocké et à quel endroit ?
        int i = 0;
        for(i = 0; i<3; i++)
        {
          if(m_elements.getCS1().getStockage(i) ==0 )
          {
            m_elements.getCS1().stock(i,m_nbrobot,req.number_order,activity::CS1);
            m_elements.getCS1().majStockID(i,1);
            break;
          }
          else if(m_elements.getCS2().getStockage(i+3) ==0 )
          {
            m_elements.getCS2().stock(i+3,m_nbrobot,req.number_order,activity::CS1);
            m_elements.getCS2().majStockID(i+3,1);
            break;
          }
          else
          {
            if(i == 3 ) ROS_ERROR("ERROR: no more place to stock ");
          }
        }
      }
      break;
      case orderRequest::UNCAP:
      // proposition : m_elements.uncap(req.parameter); + msgToGT en dehors de cette fonction
      // TODO: fonction qui retourne la CS correspondant au paramètre de couleur
      switch(req.parameter)   // à verifier? chaque CS à des capscat spécifiques
      {
        case orderRequest::BLACK :
        if(m_elements.getCS1().getBlackCap() != 0)        m_elements.getCS1().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getBlackCap() != 0)   m_elements.getCS2().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
        case orderRequest::GREY :
        if(m_elements.getCS1().getGreyCap() != 0)         m_elements.getCS1().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS1);
        else if(m_elements.getCS2().getGreyCap() != 0)    m_elements.getCS2().uncap(req.parameter,m_nbrobot,req.number_order,activity::CS2);
        break;
      }
      break;
      case orderRequest::DESTOCK:
      if(req.id >= 0 && req.id < 3)
      {
        m_elements.getCS1().destock(req.id,m_nbrobot,req.number_order,activity::CS1);
        m_elements.getCS1().majStockID(req.id, 0);
        res.accepted = true;
        res.needToResendOrder = false;
      }
      else if(req.id >= 3 && req.id < 6)
      {
        m_elements.getCS2().destock(req.id,m_nbrobot,req.number_order,activity::CS2);
        m_elements.getCS2().majStockID(req.id, 0);
        res.accepted = true;
        res.needToResendOrder = false;
      }
      else
      {
        ROS_ERROR("ERROR: req.id is not between 0 and 5 ");
        res.accepted =false;
        res.needToResendOrder = true;
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
          // TODO: gérer les cas d'erreurs de going
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
          // déterminer second coin zone (le plus accessible), avec angle différent du premier
          // NB: en cas de mur, pouvoir déterminer un point légèrement décalé
          if (!knownMachineInZone(req.id) && !m_ls->haveAllTheMachines())
          {
            ROS_INFO("No known Machine in this area %d", req.id);
            // TODO: choix judicieux du coin à déterminer
            interpretationZone(req.id, BOTTOM_RIGHT);
            //ROS_INFO("Point Target BottRight (%f, %f) theta: %f", m_explo_target.x, m_explo_target.y, m_explo_target.theta);
            ROS_INFO("Point Target Bottom Right");

            // Se rendre au second coin zone
            // TODO: gérer les cas d'erreurs de going
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
          if (!machineIsDs(machineSideId))
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

        // TODO: Le going ci-dessus peut avoir demandé un déplacement très
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
              if (!machineIsDs(machineSideId))
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
        if(!common_utils::exists(machineSideId))
        {
            m_ls->spin();
            for(auto &it : m_ls->machines())
            {
                if (it.zone == req.id && it.orientationOk)
                {
                    geometry_msgs::Pose2D robotPose = m_rp->getPose2D();
                    geometry_msgs::Pose2D machinePose = it.pose;
                    geometry_msgs::Pose2D robotPoseInMachineFrame = robotPose;
                    double machineTheta = machinePose.theta;

                    double tx = -(machinePose.x * cos(machineTheta) - machinePose.y*sin(machineTheta));
                    double ty = -(machinePose.x * sin(machineTheta) + machinePose.y*cos(machineTheta));

                    robotPoseInMachineFrame.x = robotPose.x * cos(machineTheta) + robotPose.y * sin(machineTheta) + tx;
                    robotPoseInMachineFrame.y = - robotPose.x * sin(machineTheta) + robotPose.y * cos(machineTheta) + ty;

                    if (robotPoseInMachineFrame.y < 0)
                    {
                        //input
                        machineSideId = it.idIn;
                    }
                    else
                    {
                        //output
                        machineSideId = it.idOut;
                    }


                    break;
                }
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
        if(isInput(machineSideId) && !machineIsDs(machineSideId))
        {
          // Si OUI
          machine->majEntry(firstSidePoint);
          machine->majExit(secondSidePoint);
          ROS_ERROR("I see an input of a machine with the angle %f", machine->getCenterMachine().theta);

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
          if(!common_utils::exists(machineSideId))
          {
              m_ls->spin();
              for(auto &it : m_ls->machines())
              {
                  if (it.zone == req.id && it.orientationOk)
                  {
                      geometry_msgs::Pose2D robotPose = m_rp->getPose2D();
                      geometry_msgs::Pose2D machinePose = it.pose;
                      geometry_msgs::Pose2D robotPoseInMachineFrame = robotPose;
                      double machineTheta = machinePose.theta;

                      double tx = -(machinePose.x * cos(machineTheta) - machinePose.y*sin(machineTheta));
                      double ty = -(machinePose.x * sin(machineTheta) + machinePose.y*cos(machineTheta));

                      robotPoseInMachineFrame.x = robotPose.x * cos(machineTheta) + robotPose.y * sin(machineTheta) + tx;
                      robotPoseInMachineFrame.y = - robotPose.x * sin(machineTheta) + robotPose.y * cos(machineTheta) + ty;

                      if (robotPoseInMachineFrame.y < 0)
                      {
                          //input
                          machineSideId = it.idIn;
                      }
                      else
                      {
                          //output
                          machineSideId = it.idOut;
                      }


                      break;
                  }
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
         if(!common_utils::exists(machineSideId))
         {
             ROS_WARN("unable to see the Id : Try fall back to the Id from cartographie");
             m_ls->spin();
             for(auto &it : m_ls->machines())
             {
                 if (it.zone == req.id && it.orientationOk)
                 {
                     geometry_msgs::Pose2D robotPose = m_rp->getPose2D();
                     geometry_msgs::Pose2D machinePose = it.pose;
                     geometry_msgs::Pose2D robotPoseInMachineFrame = robotPose;
                     double machineTheta = machinePose.theta;

                     double tx = -(machinePose.x * cos(machineTheta) - machinePose.y*sin(machineTheta));
                     double ty = -(machinePose.x * sin(machineTheta) + machinePose.y*cos(machineTheta));

                     robotPoseInMachineFrame.x = robotPose.x * cos(machineTheta) + robotPose.y * sin(machineTheta) + tx;
                     robotPoseInMachineFrame.y = - robotPose.x * sin(machineTheta) + robotPose.y * cos(machineTheta) + ty;

                     ROS_DEBUG_STREAM("Id determination : Robot is at " << robotPose);
                     ROS_DEBUG_STREAM("Id determination : Machine is at " << machinePose);
                     if (robotPoseInMachineFrame.y < 0)
                     {
                         //input
                         ROS_WARN_STREAM("Id determination : Robot is at the input side (" << robotPoseInMachineFrame.x << ", " << robotPoseInMachineFrame.y << ")");
                         machineSideId = it.idIn;
                     }
                     else
                     {
                         //output
                         ROS_WARN_STREAM("Id determination : Robot is at the Output side (" << robotPoseInMachineFrame.x << ", " << robotPoseInMachineFrame.y << ")");
                         machineSideId = it.idOut;
                     }


                     fa_c.setBestFirstRotation(getBestRotationForFA(machineTheta, robotPoseInMachineFrame));

                     break;
                 }
             }
         }

         if(machineIsDs(machineSideId))
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
    m_msg = m_elements.getBS().msgToGT(m_nbrobot,activity::END,activity::NONE,req.id);
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

uint8_t GtServerSrv::getBestRotationForFA(float machineTheta, geometry_msgs::Pose2D robotPoseInMachineFrame)
{
    constexpr float artagXoffset = 0.025;
    constexpr float artagYoffset = 0.175;

    geometry_msgs::Pose2D artagPoseInMachineFrame;
    artagPoseInMachineFrame.x = -artagXoffset;
    float angle;

    // OUTPUT
    if (robotPoseInMachineFrame.y > 0)
    {
        artagPoseInMachineFrame.y = artagYoffset;
        artagPoseInMachineFrame.theta = -machineTheta;

    }
    // INPUT
    else
    {
        artagPoseInMachineFrame.y = -artagYoffset;
        artagPoseInMachineFrame.theta = machineTheta;

    }

    if (geometry_utils::angle(robotPoseInMachineFrame, artagPoseInMachineFrame) >= robotPoseInMachineFrame.theta)
    {
        return final_approach_msg::FinalApproachingGoal::LEFT;
    }
    else
    {
        return final_approach_msg::FinalApproachingGoal::RIGHT;
    }
}