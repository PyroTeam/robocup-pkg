#include "EKF_class.h"
#include "ros/ros.h"

using namespace Eigen;

EKF::EKF(){
  m_xMean.conservativeResize(3);
  m_xMean.setZero();

  m_xPredicted.conservativeResize(3);
  m_xPredicted.setZero();

  m_cmdVel.conservativeResize(3);
  m_cmdVel.setZero();

  m_P.conservativeResize(3,3);
  m_P.setZero();

  m_P_prev.conservativeResize(3,3);
  m_P_prev.setZero();

  m_temps = ros::Time::now();
}

void EKF::set(){
  m_xMean(0) = m_odomRobot.x;
  m_xMean(1) = m_odomRobot.y;
  m_xMean(2) = m_odomRobot.theta;
}

int EKF::getZone(geometry_msgs::Pose2D m){
  //côté droit
  if(m.x >= 0 && m.y >= 0 && m.x <= 6 && m.y < 6) {
    int w = int(m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h;
  }
  //côté gauche
  else if (m.x < 0 && m.y >= 0 && m.x <= -6 && m.y < 6) {
    int w = int(-m.x/2);
    int h = int(m.y/1.5)+1;

    return w*4 + h + 12;
  }
  else {
    return 0;
  }
}

geometry_msgs::Pose2D EKF::getCenter(int zone){
  geometry_msgs::Pose2D c;

  // Right side
  if(zone<=12) {
    c.x = ((zone-1)/4)*2 + 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }
  // Left side
  else if (zone<=24) {
    zone -=12;
    c.x = -((zone-1)/4)*2 - 1;
    c.y = ((zone-1)%4)*1.5 + 0.75;
  }

  return c;
}

double EKF::dist(geometry_msgs::Pose2D c, geometry_msgs::Pose2D m){
  return (m.x - c.x)*(m.x - c.x) + (m.y - c.y)*(m.y - c.y);
}

int EKF::machineToArea(geometry_msgs::Pose2D m){
  int zone = getZone(m);
  std::cout << "machine (" << m.x << "," << m.y << ") en zone " << zone << std::endl;
  if ((zone != 0) && (dist(m,getCenter(zone)) <= 0.36)){
  //si on est dans le cercle de centre le centre de zone et de rayon 0.6 m
  //pour éviter un sqrt() on met le seuil au carré
    return zone;
  }
  else {
    return 0;
  }
}

void EKF::odomCallback(const nav_msgs::Odometry& odom){
  m_odomRobot.x = odom.pose.pose.position.x;
  m_odomRobot.y = odom.pose.pose.position.y;
  m_odomRobot.theta = tf::getYaw(odom.pose.pose.orientation);

  m_cmdVel(0) = odom.twist.twist.linear.x;
  m_cmdVel(1) = odom.twist.twist.linear.y;
  m_cmdVel(2) = odom.twist.twist.angular.z;
}

void EKF::machinesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  m_tabLandmarks.clear();
  for (auto &it : machines->landmarks){
    //geometry_msgs::Pose2D posLaser;
    //posLaser.x     = it.x;
    //posLaser.y     = it.y;
    //posLaser.theta = it.theta;

    //geometry_msgs::Pose2D pDansRepereGlobal = RobotToGlobal(LaserToRobot(posLaser));
    //int i = machineToArea2(pDansRepereGlobal);
    //if (i != 0)
    //{
      //std::cout << "la machine (" << pDansRepereGlobal.x << "," << pDansRepereGlobal.x << ")" << " appartient à la zone " << i << std::endl;
      m_tabLandmarks.push_back(it);
    //}
  }
}

void EKF::machinesVuesCallback(const deplacement_msg::LandmarksConstPtr& machines){
  m_tabMachines.clear();
  for (auto &it : machines->landmarks){
    geometry_msgs::Pose2D p = RobotToGlobal(LaserToRobot(it));

    int zone = machineToArea(p);
    if(zone!=0){
      m_tabMachines.push_back(p);
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

    //std::cout << "le robot est en (" << m_xPredicted(0) << "," << m_xPredicted(1) << ")" << std::endl;
    
    m_scan.push_back(pDansRepereGlobal);
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
  rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot;
  m(2,2) = 1;

  before(0) = PosLaser.x;
  before(1) = PosLaser.y;
  before(2) = 1;

  after = m*before;

  p.x = after(0) ;
  p.y = after(1);
  p.theta = PosLaser.theta;

  return p;
}

VectorXd EKF::RobotToLaser(VectorXd PosRobot){
  Matrix3d m;
  m.setZero();
  Vector3d before;
  Vector3d after;
  
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(M_PI_2);
  m.topLeftCorner(2,2) = rot.transpose();

  //translation
  Vector2d t;
  t(0) = 0;
  t(1) = 0.1;
  Vector2d ti;
  ti = -rot.transpose()*t;                        

  m(2,2) = 1;

  before(0) = PosRobot(0);
  before(1) = PosRobot(1);
  before(2) = 1;

  after = m*before;

  after(2) = PosRobot(2);

  return after;
}

geometry_msgs::Pose2D EKF::RobotToGlobal(geometry_msgs::Pose2D p){
  Vector3d before, after;
  Matrix3d m;
  geometry_msgs::Pose2D p2;
  double angle = m_xMean(2);

  before(0) = p.x;
  before(1) = p.y;
  before(2) = 1; //toujours 1 ici !

  m.setZero();
  //translation
  m(0,2) = m_xMean(0);
  m(1,2) = m_xMean(1);
  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(angle - M_PI_2);
  m.topLeftCorner(2,2) = rot;

  m(2,2) = 1;

  after = m*before;

  p2.x   = after(0);
  p2.y   = after(1);
  p2.theta = p.theta/* - angle*/;

  return p2;
}

VectorXd EKF::GlobalToRobot(VectorXd p){
  Vector3d before, after;
  Matrix3d m;
  double angle = m_xMean(2);

  before(0) = p(0);
  before(1) = p(1);
  before(2) = 1; //toujours 1 ici !

  m.setZero();

  //rotation
  Matrix2d rot;
  rot = Rotation2Dd(angle - M_PI_2);
  m.topLeftCorner(2,2) = rot.transpose();

  //translation
  Vector2d t;
  t(0) = m_xMean(0);
  t(1) = m_xMean(1);
  Vector2d ti;
  ti = -rot.transpose()*t;    

  m(2,2) = 1;

  after = m*before;

  after(2) = p(2);

  return after;
}

void EKF::addMachine(geometry_msgs::Pose2D m){
  //on redimensionne m_xMean et m_P pour accueillir la nouvelle machines
  m_xMean.conservativeResize(m_xMean.rows() + 3);
  m_xPredicted.conservativeResize(m_xMean.rows());
  m_P.conservativeResize(m_P.rows()+3,m_P.cols()+3);

  //on remplit avec les coordonnées de la nouvelle machine
  m_xMean(m_xMean.rows()-3) = m.x;
  m_xMean(m_xMean.rows()-2) = m.y;
  m_xMean(m_xMean.rows()-1) = m.theta;

  //std::cout << "machine ajoutée : " << m_xMean.block(m_xMean.rows()-3,0,3,1) << std::endl;

  //calcul de tous les PLi
  //initialisation des PLi à 0
  m_P.block(m_P.rows()-3, 0,3, m_P.cols()).setZero();
  m_P.block(0, m_P.cols()-3, m_P.rows(),3).setZero();

  for (int j = 0; j < m_xMean.rows(); j = j + 3){
    //position de la nouvelle machines par rapport au robot
    //et à toutes les autres
    double x     = m_xMean(m_xMean.rows()-3) - m_xMean(j  );
    double y     = m_xMean(m_xMean.rows()-2) - m_xMean(j+1);
    double theta = m_xMean(m_xMean.rows()-1) - m_xMean(j+2);

    m_P(m_P.rows()-3, j  ) = x;
    m_P(m_P.rows()-2, j+1) = y;
    m_P(m_P.rows()-1, j+2) = theta;

    m_P(j  , m_P.cols()-3) = x;
    m_P(j+1, m_P.cols()-2) = y;
    m_P(j+2, m_P.cols()-1) = theta;

    m_P(j  , j  ) = m_xMean(j  ) - m_xMean(0);
    m_P(j+1, j+1) = m_xMean(j+1) - m_xMean(1);
    m_P(j+2, j+2) = m_xMean(j+2) - m_xMean(2);
  }
}

int EKF::checkStateVector(geometry_msgs::Pose2D machine){
  //std::cout << "taille de m_xMean :" << m_xMean.rows() << "\n" << std::endl;
  for (int i = 3; i < m_xMean.rows(); i=i+3){
    if ((std::abs(machine.x     - m_xMean(i  )) < 0.5) &&
        (std::abs(machine.y     - m_xMean(i+1)) < 0.5) &&
        (std::abs(machine.theta - m_xMean(i+2)) < 0.08)){
      return i;
    }
  }

  return 0;
}

MatrixXd EKF::buildPm(int i){
  MatrixXd Pm(m_P.rows(),m_P.cols());
  Pm.setZero();

  Pm.block(0,0,3,3) = m_P_prev.block(0,0,3,3);
  Pm.block(i,i,3,3) = m_P_prev.block(i,i,3,3);
  Pm.block(0,i,3,3) = m_P_prev.block(0,i,3,3);
  Pm.block(i,0,3,3) = m_P_prev.block(i,0,3,3);

  //std::cout << "Pm :" << Pm << std::endl;   

  return Pm;
}

void EKF::updateP(const MatrixXd &Pm, int i){
  m_P.block(0,0,3,3) = Pm.block(0,0,3,3);
  m_P.block(i,i,3,3) = Pm.block(i,i,3,3);
  m_P.block(0,i,3,3) = Pm.block(0,i,3,3);
  m_P.block(i,0,3,3) = Pm.block(i,0,3,3);   
}

void EKF::updatePprev(const MatrixXd &Pm, int i){
  m_P_prev.block(0,0,3,3) = Pm.block(0,0,3,3);
  m_P_prev.block(i,i,3,3) = Pm.block(i,i,3,3);
  m_P_prev.block(0,i,3,3) = Pm.block(0,i,3,3);
  m_P_prev.block(i,0,3,3) = Pm.block(i,0,3,3);   
}

MatrixXd EKF::buildH2(geometry_msgs::Pose2D p, int taille, int i){
  MatrixXd H(3,taille);
  H.setZero();
  H.block(0,0,3,3) = MatrixXd::Identity(3,3);
  H.block(0,i,3,3) = MatrixXd::Identity(3,3);

  return H;
}

void EKF::prediction(){
  //std::cout << "prediction" << std::endl;

  //calcul de la période pour la prédiction
  ros::Duration duree = ros::Time::now() - m_temps;
  double periode = duree.toSec();

  //calcul de la position du robot pour l'instant n+1
  m_xPredicted(0) = m_xMean(0) + periode*(cos(m_xMean(2))*m_cmdVel(0)-sin(m_xMean(2))*m_cmdVel(1));
  m_xPredicted(1) = m_xMean(1) + periode*(sin(m_xMean(2))*m_cmdVel(0)-cos(m_xMean(2))*m_cmdVel(1));
  m_xPredicted(2) = m_xMean(2) + periode*m_cmdVel(2);

  m_xMean.block(0,0,3,1) = m_xPredicted.block(0,0,3,1);

  MatrixXd Fx = MatrixXd::Identity(m_P.rows(),m_P.cols());
  Fx(0,2) =  m_cmdVel(0)*cos(m_xMean(2))*periode;
  Fx(1,2) = -m_cmdVel(1)*sin(m_xMean(2))*periode;

  //mise à jour de m_P
  m_P_prev = Fx*m_P*(Fx.transpose());
  //mise à jour du temps
  m_temps = ros::Time::now();
}

void EKF::correction(geometry_msgs::Pose2D p, int i){
  //std::cout << "correction\n" << std::endl;

  int taille = m_P.rows();

  //calcul de z
  Vector3d z;
  VectorXd mVect(3);
  mVect(0) = p.x;
  mVect(1) = p.y;
  mVect(2) = p.theta;
  //VectorXd positionMachine = GlobalToRobot(RobotToLaser(mVect));
  z = mVect - m_xMean.block(i,0,3,1);
  //std::cout << "mesure = \n" << mVect << std::endl;
  //std::cout << "machine = \n" << m_xMean.block(i,0,3,1) << std::endl;
  //std::cout << "z = \n" << z << std::endl;

  //calcul de H
  MatrixXd H;
  H = buildH2(p,taille,i);
  //std::cout << "H = \n" << H << "\n" <<  std::endl;

  //calcul de R
  MatrixXd R(3,3);
  R.setZero();
  R(0,0) = 0.05;
  R(1,1) = 0.05;
  R(2,2) = 0.1;
  //std::cout << "R = \n" << R << "\n" <<  std::endl;

  //calcul de Pm
  MatrixXd Pm = buildPm(i);
  //std::cout << "Pm =" << Pm << "\n" <<  std::endl;

  //calcul de Z
  MatrixXd Z(3,3);
  Z.setZero();
  Z = H*Pm*H.transpose() + R;
  //std::cout << "Z = \n" << Z << "\n" <<  std::endl;

  //calcul du gain de Kalman
  MatrixXd K;
  K.setZero();
  K = Pm*H.transpose()*Z.inverse();
  //std::cout << "K = \n" << K << "\n" <<  std::endl;


  //mise à jour du vecteur m_xMean
  m_xMean.block(0,0,3,1) += (K*z).block(0,0,3,1);
  //m_xMean.block(i,0,3,1) += (K*z).block(i,0,3,1);

  //std::cout << "je corrige la machine (" << m.x << "," << m.y << ")" << std::endl;
  //std::cout << "la nouvelle position :(" << m_xMean(i) << "," << m_xMean(i+1) << ")" << std::endl;
  //std::cout << "xMean après : \n" << m_xMean << std::endl;

  //mise à jour de la matrice m_P
  MatrixXd tmp = MatrixXd::Identity(Pm.rows(),Pm.cols()) - K*H; 
  Pm = tmp*Pm;
  //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;
  updateP(Pm, i);
}

void EKF::printZones(){
  for (int i = 0; i < m_zones.size(); ++i){
    std::cout << "machine en zone " << m_zones[i] << std::endl;
  }
}

void EKF::fillMachines(){
  for (auto &it : m_tabMachines){
    addMachine(it);
  }
}

VectorXd EKF::getXmean() {
  return m_xMean;
}
VectorXd EKF::getXpredicted() {
  return m_xPredicted;
}
std::vector<int> EKF::getZones() {
  return m_zones;
}
std::vector<geometry_msgs::Pose2D> EKF::getScan(){
  return m_scan;
}
std::vector<geometry_msgs::Pose2D> EKF::getTabMachines() {
  return m_tabMachines;
}