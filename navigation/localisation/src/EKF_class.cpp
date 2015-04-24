#include "EKF_class.h"
#include "ros/ros.h"

using namespace Eigen;

EKF::EKF(std::string s, int num){
  /*int color = 0;
  if (s == "cyan"){
    color = 1;
  }
  else {
    color = -1;
  }

  m_initRobot.theta =  0.0;
  m_initRobot.y   = -0.5;

  switch(n){
    case 1 :
      m_initRobot.x = color*3.5;
    break;
    case 2 :
      m_initRobot.x = color*4.5;
    break;
    case 3 :
      m_initRobot.x = color*5.5;
    break;
    default :
    break;
  }*/

  m_initRobot.x     = 0.0;
  m_initRobot.y     = 0.0;
  m_initRobot.theta = 0.0;

  m_xMean.conservativeResize(3);
  m_xMean.setZero();
  m_P.conservativeResize(3,3);
  m_P.setZero();

  m_temps = ros::Time::now();
}

int EKF::machineToArea(geometry_msgs::Pose2D m){
  if (m.x > 0.0 && m.x < 2.0 && m.y > 0.0 && m.y < 1.5) return 1;
  if (m.x > 0.0 && m.x < 2.0 && m.y > 1.5 && m.y < 3.0) return 2;
  if (m.x > 0.0 && m.x < 2.0 && m.y > 3.0 && m.y < 4.5) return 3;
  if (m.x > 0.0 && m.x < 2.0 && m.y > 4.5 && m.y < 6.0) return 4;

  if (m.x > 2.0 && m.x < 4.0 && m.y > 0.0 && m.y < 1.5) return 5;
  if (m.x > 2.0 && m.x < 4.0 && m.y > 1.5 && m.y < 3.0) return 6;
  if (m.x > 2.0 && m.x < 4.0 && m.y > 3.0 && m.y < 4.5) return 7;
  if (m.x > 2.0 && m.x < 4.0 && m.y > 4.5 && m.y < 6.0) return 8;

  if (m.x > 4.0 && m.x < 6.0 && m.y > 0.0 && m.y < 1.5) return 9;
  if (m.x > 4.0 && m.x < 6.0 && m.y > 1.5 && m.y < 3.0) return 10;
  if (m.x > 4.0 && m.x < 6.0 && m.y > 3.0 && m.y < 4.5) return 11;
  if (m.x > 4.0 && m.x < 6.0 && m.y > 4.5 && m.y < 6.0) return 12;

  if (m.x > -2.0 && m.x < 0.0 && m.y > 0.0 && m.y < 1.5) return 13;
  if (m.x > -2.0 && m.x < 0.0 && m.y > 1.5 && m.y < 3.0) return 14;
  if (m.x > -2.0 && m.x < 0.0 && m.y > 3.0 && m.y < 4.5) return 15;
  if (m.x > -2.0 && m.x < 0.0 && m.y > 4.5 && m.y < 6.0) return 16;

  if (m.x > -4.0 && m.x < -2.0 && m.y > 0.0 && m.y < 1.5) return 17;
  if (m.x > -4.0 && m.x < -2.0 && m.y > 1.5 && m.y < 3.0) return 18;
  if (m.x > -4.0 && m.x < -2.0 && m.y > 3.0 && m.y < 4.5) return 19;
  if (m.x > -4.0 && m.x < -2.0 && m.y > 4.5 && m.y < 6.0) return 20;

  if (m.x > -6.0 && m.x < -4.0 && m.y > 0.0 && m.y < 1.5) return 21;
  if (m.x > -6.0 && m.x < -4.0 && m.y > 1.5 && m.y < 3.0) return 22;
  if (m.x > -6.0 && m.x < -4.0 && m.y > 3.0 && m.y < 4.5) return 23;
  if (m.x > -6.0 && m.x < -4.0 && m.y > 4.5 && m.y < 6.0) return 24;
  else                                                    return 0;
}

void EKF::odomCallback(const nav_msgs::Odometry& odom){
  m_odomRobot.x = odom.pose.pose.position.x;
  m_odomRobot.y = odom.pose.pose.position.y;
  m_odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);

  m_cmdVel(0) = odom.twist.twist.linear.x;
  m_cmdVel(1) = odom.twist.twist.linear.y;
  m_cmdVel(2) = odom.twist.twist.angular.z;

  cmdVelDansGlobal(m_cmdVel(2));
}

void EKF::machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  m_tabMachines.clear();
  for (auto &it : machines->landmarks){
    geometry_msgs::Pose2D posLaser;
    posLaser.x     = it.x;
    posLaser.y     = it.y;
    posLaser.theta = it.theta;
    //changement de base vers le repère du robot
    geometry_msgs::Pose2D p = LaserToRobot(posLaser);
    //changment de base dans le repère global

    geometry_msgs::Pose2D pDansRepereGlobal = RobotToGlobal(p);
    int i = machineToArea(pDansRepereGlobal);
    if (i != 0)
    {
      //std::cout << "la machine (" << pDansRepereGlobal.x << "," << pDansRepereGlobal.x << ")" << " appartient à la zone " << i << std::endl;
      m_tabMachines.push_back(pDansRepereGlobal);
    }
  }
}

void EKF::laserCallback(const deplacement_msg::LandmarksConstPtr& laser){
  m_scan.clear();
  for (auto &it : laser->landmarks){
    geometry_msgs::Pose2D posLaser;
    posLaser.x     = it.x;
    posLaser.y     = it.y;
    //changement de base vers le repère du robot
    geometry_msgs::Pose2D p = LaserToRobot(posLaser);
    //changment de base dans le repère global
    geometry_msgs::Pose2D pDansRepereGlobal = RobotToGlobal(p);
    
    m_scan.push_back(pDansRepereGlobal);
  }
}

void EKF::correctAngle(double &angle){
  double dx = 0;
  while (angle > M_PI || angle <= -M_PI){
    if (angle > M_PI){
      dx = M_PI - angle;
      angle = -M_PI + dx;
    }
    else if (angle <= -M_PI){
      dx = -M_PI - angle;
      angle = M_PI - dx;
    }
  }
}

void EKF::correctStateVector(){
  for (int i = 2; i < m_xMean.rows(); i=i+3){
    correctAngle(m_xMean(i));
  }
}

geometry_msgs::Pose2D EKF::LaserToRobot(geometry_msgs::Pose2D PosLaser){
  geometry_msgs::Pose2D p;
  Matrix3d m;
  m.setZero();
  Vector3d before;
  Vector3d after;

  //translation
  m(1,2) = 0.1;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(-M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  p.x = after(0) ;
  p.y = after(1) ;
  p.theta = atan2(PosLaser.theta,1);

  return p;
}

geometry_msgs::Pose2D EKF::RobotToGlobal(geometry_msgs::Pose2D p){
  Vector3d before, after;
  Matrix3d m;
  double angle = m_odomRobot.theta;
  correctAngle(angle);

  before(0) = p.x;
  before(1) = p.y;
  before(2) = 1; //toujours 1 ici !

  m.setZero();
  //translation
  m(0,2) = m_xMean(0);
  m(1,2) = m_xMean(1);
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(angle);
  m.topLeftCorner(2,2) = rot;

  m(2,2) = 1;

  after = m*before;

  p.x   = after(0);
  p.y   = after(1);
  p.theta = angle;

  return p;
}

void EKF::cmdVelDansGlobal(double angle){
  Matrix3d m;
  m.setZero();

  Matrix2d rot;
  rot = Rotation2Dd(angle);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  m_cmdVel = m*m_cmdVel;
}

void EKF::addMachine(geometry_msgs::Pose2D machine){
  //on redimensionne m_xMean et m_P pour accueillir la nouvelle machines
  m_xMean.conservativeResize(m_xMean.rows() + 3);
  m_P.conservativeResize(m_P.rows()+3,m_P.cols()+3);

  //on remplit avec les coordonnées de la nouvelle machine
  m_xMean(m_xMean.rows()-3) = machine.x;
  m_xMean(m_xMean.rows()-2) = machine.y;
  m_xMean(m_xMean.rows()-1) = machine.theta;
  correctAngle(m_xMean(m_xMean.rows()-1));

  //calcul de tous les PLi
  //initialisation des PLi à 0
  m_P.block(m_P.rows() - 3, 0, 3, m_P.cols()).setZero();
  m_P.block(0, m_P.cols() - 3, m_P.rows(), 3).setZero();

  for (int j = 0; j < m_xMean.rows(); j = j + 3){
    //position de la nouvelle machines par rapport au robot
    //et à toutes les autres
    double x     = m_xMean(m_xMean.rows()-3) - m_xMean(j  );
    double y     = m_xMean(m_xMean.rows()-2) - m_xMean(j+1);
    double theta = m_xMean(m_xMean.rows()-1) - m_xMean(j+2);

    m_P(m_P.rows()-3, j  ) = x;
    m_P(m_P.rows()-2, j+1) = y;
    m_P(m_P.rows()-1, j+2) = theta;

    m_P(j  , m_P.rows()-3) = x;
    m_P(j+1, m_P.rows()-2) = y;
    m_P(j+2, m_P.rows()-1) = theta;

    m_P(j  , j  ) = m_xMean(j  ) - m_xMean(0);
    m_P(j+1, j+1) = m_xMean(j+1) - m_xMean(1);
    m_P(j+2, j+2) = m_xMean(j+2) - m_xMean(2);
  }
}

int EKF::checkStateVector(geometry_msgs::Pose2D machine){
  //std::cout << "taille de m_xMean :" << m_xMean.rows() << "\n" << std::endl;
  for (int i = 3; i < m_xMean.rows(); i=i+3){
    double d = sqrt((machine.x-m_xMean(i))*(machine.x-m_xMean(i)) +
                    (machine.y-m_xMean(i+1))*(machine.y-m_xMean(i+1)));
    //std::cout << "la distance entre machines est de " << d << std::endl;
    if (d < 0.5) return i;
  }

  return 0;
}

MatrixXd EKF::buildPm(int i){
  MatrixXd Pm(6,6);
  Pm.setZero();

  Pm.block(0,0,3,3) = m_P_prev.block(0,0,3,3);
  Pm.block(3,3,3,3) = m_P_prev.block(i,i,3,3);
  Pm.block(0,3,3,3) = m_P_prev.block(0,i,3,3);
  Pm.block(3,0,3,3) = m_P_prev.block(i,0,3,3);
  std::cout << "Pm :" << Pm << std::endl;   

  return Pm;
}

void EKF::updateP(const MatrixXd &Pm, int i){
  m_P.block(0,0,3,3) = Pm.block(0,0,3,3);
  m_P.block(i,i,3,3) = Pm.block(3,3,3,3);
  m_P.block(0,i,3,3) = Pm.block(0,3,3,3);
  m_P.block(i,0,3,3) = Pm.block(3,0,3,3);   
}

MatrixXd EKF::buildH(int i){
  MatrixXd H(3,6);
  H.setZero();
  H.block(0,0,3,3) = MatrixXd::Identity(3,3);
  H(0,3) = m_xPredicted(0) - m_xMean(i  );
  H(1,4) = m_xPredicted(1) - m_xMean(i+1);
  H(2,5) = m_xPredicted(2) - m_xMean(i+2);

  //std::cout << "H = \n" << H << "\n" <<  std::endl;

  return H;
}

MatrixXd EKF::buildH2(int taille, int i){
  MatrixXd H(3,taille);
  H.setZero();
  H.block(0,0,3,3) = MatrixXd::Identity(3,3);
  H(0,i  ) = m_xMean(0) - m_xMean(i  );
  H(1,i+1) = m_xMean(1) - m_xMean(i+1);
  H(2,i+2) = m_xMean(2) - m_xMean(i+2);

  return H;
}

void EKF::prediction(){
  std::cout << "prediction" << std::endl;

  //calcul de la période pour la prédiction
  ros::Duration duree = ros::Time::now() - m_temps;
  double periode = duree.toSec();

  correctAngle(m_xMean(2));
  cmdVelDansGlobal(m_xMean(2));

  //calcul de la position du robot pour l'instant n+1
  m_xPredicted = m_xMean.topLeftCorner(3,1) + periode*m_cmdVel;

  MatrixXd Fx = MatrixXd::Identity(m_P.rows(),m_P.cols());/*
  Fx(0,0) = m_xPredicted(0) - m_xMean(0);
  Fx(1,1) = m_xPredicted(1) - m_xMean(1);
  Fx(2,2) = m_xPredicted(2) - m_xMean(2);*/
  //std::cout << "Fx = \n" << Fx << std::endl;

  //m_xMean.topLeftCorner(3,1) = m_xPredicted;
  //correctAngle(m_xMean(2));

  //mise à jour de m_P
  m_P_prev = Fx*m_P*(Fx.transpose());
  //mise à jour du temps
  m_temps = ros::Time::now();
}

void EKF::correction(geometry_msgs::Pose2D p, int i){
  std::cout << "correction\n" << std::endl;

  //int taille = m_P.rows();

  //calcul de z
  VectorXd z(6);
  z.setZero();
  z.block(0,0,3,1) = m_xPredicted - m_xMean.block(m_xMean.rows()-3,0,3,1);
  z(3) = p.x     - m_xPredicted(0);
  z(4) = p.y     - m_xPredicted(1);
  z(5) = p.theta - m_xPredicted(2);
  //std::cout << "z = \n" << z << "\n" << std::endl;

  //calcul de H
  MatrixXd H(3,6);
  H = buildH(i);
  //std::cout << "H = \n" << H << "\n" <<  std::endl;

  //calcul de Ht
  MatrixXd Ht(6,3);
  Ht.block(0,0,3,3) = H.block(0,0,3,3).transpose();
  Ht.block(3,0,3,3) = H.block(0,3,3,3).transpose();
  //std::cout << "Ht = \n" << Ht << "\n" <<  std::endl;

  //calcul de R
  MatrixXd R(3,3);
  R.setZero();
  R(0,0) = 0.1;
  R(1,1) = 0.1;
  R(2,2) = 0.1;
  //std::cout << "R = \n" << R << "\n" <<  std::endl;

  //calcul de Pm
  MatrixXd Pm = buildPm(i);
  //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;

  //calcul de Z
  MatrixXd Z(3,3);
  Z.setZero();
  Z = H*Pm*Ht + R;
  //std::cout << "Z = \n" << Z << "\n" <<  std::endl;

  //calcul du gain de Kalman
  MatrixXd K;
  K.setZero();
  K = Pm*Ht*(Z.inverse());
  //std::cout << "K = \n" << K << "\n" <<  std::endl;

  //mise à jour du vecteur m_xMean
  m_xMean.block(0,0,3,1) += K.block(0,0,3,3)*z.block(0,0,3,1);
  m_xMean.block(i,0,3,1) += K.block(3,0,3,3)*z.block(3,0,3,1);

  std::cout << "je corrige la machine (" << p.x << "," << p.y << ")" << std::endl;
  std::cout << "la nouvelle position :(" << m_xMean(i) << "," << m_xMean(i+1) << ")" << std::endl;

  correctStateVector();
  //std::cout << "position du robot corrigee = \n" <<  m_xMean.topLeftCorner(3,1) << "\n" << std::endl;

  //mise à jour de la matrice m_P
  MatrixXd tmp = MatrixXd::Identity(Pm.rows(),Pm.cols()) - K*H; 
  Pm = tmp*Pm;
  //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;
  //std::cout << "KZKt = \n" << K*Z*(K.transpose()) << "\n" <<  std::endl;
  updateP(Pm, i);
}

bool EKF::test(int area){
  for (int i = 0; i < m_zones.size(); i++){
    if (area == m_zones[i]){
      return true;
    }
  }
  return false;
}

void EKF::printZones(){
  for (int i = 0; i < m_zones.size(); ++i){
    std::cout << "machine en zone " << m_zones[i] << std::endl;
  }
}