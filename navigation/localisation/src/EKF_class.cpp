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
    if (pDansRepereGlobal.x > -6.0 &&
        pDansRepereGlobal.x <  6.0 &&
        pDansRepereGlobal.y >  0.0 &&
        pDansRepereGlobal.y <  6.0)
    {
      m_tabMachines.push_back(pDansRepereGlobal);
    }
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
  m(1,2) = 0.2;
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  p.x = after(0) ;
  p.y = after(1) ;
  //on veut l'angle machine dans [0, 2*PI]
  if (PosLaser.theta >= 0){
    p.theta = PosLaser.theta - M_PI_2;
  }
  else {
    p.theta = PosLaser.theta + M_PI_2;
  }

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
  m(0,2) = m_initRobot.x + m_odomRobot.x;
  m(1,2) = m_initRobot.y + m_odomRobot.y;
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
  std::cout << "taille de m_xMean :" << m_xMean.rows() << "\n" << std::endl;
  for (int i = 3; i < m_xMean.rows(); i=i+3){
    double d = sqrt((machine.x-m_xMean(i))*(machine.x-m_xMean(i)) +
                    (machine.y-m_xMean(i+1))*(machine.y-m_xMean(i+1)));
    std::cout << "la distance entre machines est de " << d << std::endl;
    if (d < 2.0) return i;
  }

  return 0;
}

MatrixXd EKF::buildPm(int i){
  MatrixXd Pm(6,6);
  Pm.setZero();

  Pm.block(0,0,3,3) = m_P.block(0,0,3,3);
  Pm.block(3,3,3,3) = m_P.block(i,i,3,3);
  Pm.block(0,3,3,3) = m_P.block(0,i,3,3);
  Pm.block(3,0,3,3) = m_P.block(i,0,3,3);   

  return Pm;
}

void EKF::updateP(const MatrixXd &Pm, int i){
  m_P.block(0,0,3,3) = Pm.block(0,0,3,3);
  m_P.block(i,i,3,3) = Pm.block(3,3,3,3);
  m_P.block(0,i,3,3) = Pm.block(0,3,3,3);
  m_P.block(i,0,3,3) = Pm.block(3,0,3,3);   
}

void EKF::updateXmean(const VectorXd &tmp, int i){
  m_xMean.block(0,0,3,1) = tmp.block(0,0,3,1);
  m_xMean.block(i,0,3,1) = tmp.block(3,0,3,1);
}

MatrixXd EKF::buildH(int i){
  MatrixXd H(3,6);
  H.setZero();
  H.block(0,0,3,3) = MatrixXd::Identity(3,3);
  H(0,3) = m_xMean(0) - m_xMean(i  );
  H(1,4) = m_xMean(1) - m_xMean(i+1);
  H(2,5) = m_xMean(2) - m_xMean(i+2);

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
  std::cout << "prediction\n" << std::endl;

  //calcul de la période pour la prédiction
  ros::Duration duree = ros::Time::now() - m_temps;
  double periode = duree.toSec();

  correctAngle(m_xMean(2));
  cmdVelDansGlobal(m_xMean(2));

  //calcul de la position du robot pour l'instant n+1
  m_xPredicted = m_xMean.topLeftCorner(3,1) + periode*m_cmdVel;

  MatrixXd Fx = MatrixXd::Identity(m_P.rows(),m_P.cols());
  Fx(0,0) = m_xPredicted(0) - m_xMean(0);
  Fx(1,1) = m_xPredicted(1) - m_xMean(1);
  Fx(2,2) = m_xPredicted(2) - m_xMean(2);
  //std::cout << "Fx = \n" << Fx << std::endl;

  m_xMean.topLeftCorner(3,1) = m_xPredicted;
  correctAngle(m_xMean(2));

  //mise à jour de m_P
  m_P = Fx*m_P*(Fx.transpose());
  //mise à jour du temps
  m_temps = ros::Time::now();
}

void EKF::correction(int posMachineInStateVector){
  std::cout << "correction\n" << std::endl;

  int taille = m_P.rows();

  //calcul de z
  Vector3d z;
  z.setZero();
  z = m_xPredicted - m_xMean.block(m_xMean.rows()-3,0,3,1);
  //std::cout << "z = \n" << z << "\n" << std::endl;

  //calcul de H
  MatrixXd H(3, taille);
  H = buildH(posMachineInStateVector);
  //std::cout << "H = \n" << H << "\n" <<  std::endl;

  //calcul de R
  MatrixXd R(3,3);
  R.setZero();
  R(0,0) = m_xMean(0);
  R(1,1) = m_xMean(1);
  R(2,2) = m_xMean(2);

  //calcul de Pm
  MatrixXd Pm = buildPm(posMachineInStateVector);
  //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;

  //calcul de Z
  MatrixXd Z(taille, taille);
  Z.setZero();
  Z = H*Pm*(H.transpose()) + R;
  //std::cout << "Z = \n" << Z << "\n" <<  std::endl;

  //calcul du gain de Kalman
  MatrixXd K(3, 3);
  K.setZero();
  K = Pm*(H.transpose())*(Z.inverse());
  //std::cout << "K = \n" << K << "\n" <<  std::endl;

  //mise à jour du vecteur m_xMean
  VectorXd tmp(6,1);
  tmp.block(0,0,3,1) = m_xMean.block(0,0,3,1);
  tmp.block(3,0,3,1) = m_xMean.block(posMachineInStateVector,0,3,1);
  //std::cout << "tmp = \n" << tmp << "\n" <<  std::endl;
  tmp +=  K*z;
  updateXmean(tmp, posMachineInStateVector);
  correctStateVector();
  //std::cout << "position du robot corrigee = \n" <<  m_xMean.topLeftCorner(3,1) << "\n" << std::endl;

  //mise à jour de la matrice m_P
  Pm = Pm - K*Z*(K.transpose());
  //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;
  //std::cout << "KZKt = \n" << K*Z*(K.transpose()) << "\n" <<  std::endl;
  updateP(Pm, posMachineInStateVector);
}