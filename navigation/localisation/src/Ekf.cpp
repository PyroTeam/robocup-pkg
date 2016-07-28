#include "Ekf.h"
#include "ros/ros.h"

using namespace Eigen;

EKF::EKF()
{
    m_xMean.conservativeResize(3);
    m_xMean.setZero();

    m_xPredicted.conservativeResize(3);
    m_xPredicted.setZero();

    m_P.conservativeResize(3,3);
    m_P.setZero();

    m_P_prev.conservativeResize(3,3);
    m_P_prev.setZero();

    m_temps = ros::Time::now();

    m_begin = false;
}

bool EKF::initOdom()
{
    Vector3d pose = m_poseSub.getPoseAsVector();

    if ((std::abs(pose.sum()) > 1.0))
    {
        if (m_begin == false)
        {
            m_xMean(0) = pose(0);
            m_xMean(1) = pose(1);
            m_xMean(2) = pose(2);

            m_begin = true;
        }
    }

    return m_begin;
}

bool EKF::test(int area)
{
  for (int i = 0; i < m_areas.size(); i++)
  {
    if (m_areas[i] == area)
    {
      return true;
    }
  }
  return false;
}

void EKF::machinesCallback(const deplacement_msg::LandmarksConstPtr& machines)
{
    m_mps.clear();

    static ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");
    if (tf_prefix.size() != 0)
    {
        tf_prefix += "/";
    }

    if (machines->landmarks.size() != 0)
    {
        tf::StampedTransform transform;
        try
        {
            m_tf_listener->waitForTransform(tf_prefix+"odom",machines->header.frame_id, machines->header.stamp,ros::Duration(1.0));
            m_tf_listener->lookupTransform(tf_prefix+"odom", machines->header.frame_id, machines->header.stamp, transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }

        for (auto &it : machines->landmarks)
        {
            // Changement de repère
            geometry_msgs::Pose2D center = geometry_utils::changeFrame(it, transform);

            // On enregistre la machine
            if (common_utils::getArea(center) != 0)
            {
                m_mps.push_back(it);
            }
        }
    }
}

void EKF::addMachine(const geometry_msgs::Pose2D &m)
{
    //on redimensionne m_xMean et m_P pour accueillir la nouvelle machines
    m_xMean.conservativeResize(m_xMean.rows() + 3);
    m_xPredicted.conservativeResize(m_xMean.rows());
    m_P.conservativeResize(m_P.rows()+3,m_P.cols()+3);

    //on remplit avec les coordonnées de la nouvelle machine
    m_xMean(m_xMean.rows()-3) = m.x;
    m_xMean(m_xMean.rows()-2) = m.y;
    m_xMean(m_xMean.rows()-1) = m.theta;

    std::cout << "machine ajoutée : " << m_xMean.block(m_xMean.rows()-3,0,3,1) << std::endl;

    //calcul de tous les PLi
    //initialisation des PLi à 0
    m_P.block(m_P.rows()-3, 0,3, m_P.cols()).setZero();
    m_P.block(0, m_P.cols()-3, m_P.rows(),3).setZero();

    for (int j = 0; j < m_xMean.rows(); j = j + 3)
    {
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

    m_areas.push_back(common_utils::getArea(m));
}

int EKF::checkStateVector(const geometry_msgs::Pose2D &machine)
{
    int areaMachine = common_utils::getArea(machine);

    if (m_xMean.rows() == 3 || areaMachine == 0)
    {
        return 0;
    }
    else
    {
        //std::cout << "size de m_xMean :" << m_xMean.rows() << "\n" << std::endl;
        for (int i = 3; i < m_xMean.rows(); i=i+3)
        {
            geometry_msgs::Pose2D m;
            m.x     = m_xMean(i);
            m.y     = m_xMean(i+1);
            m.theta = m_xMean(i+2);

            if (areaMachine == common_utils::getArea(m))
            {
                return i;
            }
        }
    }

    return 0;
}

MatrixXd EKF::buildPm(int i)
{
    MatrixXd Pm(m_P.rows(),m_P.cols());
    Pm.setZero();

    Pm.block(0,0,3,3) = m_P_prev.block(0,0,3,3);
    Pm.block(i,i,3,3) = m_P_prev.block(i,i,3,3);
    Pm.block(0,i,3,3) = m_P_prev.block(0,i,3,3);
    Pm.block(i,0,3,3) = m_P_prev.block(i,0,3,3);

    //std::cout << "Pm :" << Pm << std::endl;

    return Pm;
}

void EKF::updateP(const MatrixXd &Pm, int i)
{
    m_P.block(0,0,3,3) = Pm.block(0,0,3,3);
    m_P.block(i,i,3,3) = Pm.block(i,i,3,3);
    m_P.block(0,i,3,3) = Pm.block(0,i,3,3);
    m_P.block(i,0,3,3) = Pm.block(i,0,3,3);
}

void EKF::updatePprev(const MatrixXd &Pm, int i)
{
    m_P_prev.block(0,0,3,3) = Pm.block(0,0,3,3);
    m_P_prev.block(i,i,3,3) = Pm.block(i,i,3,3);
    m_P_prev.block(0,i,3,3) = Pm.block(0,i,3,3);
    m_P_prev.block(i,0,3,3) = Pm.block(i,0,3,3);
}

MatrixXd EKF::buildH2(const geometry_msgs::Pose2D &p, int size, int i)
{
    MatrixXd H(3,size);
    H.setZero();
    H.block(0,0,3,3) = MatrixXd::Identity(3,3);
    H.block(0,i,3,3) = MatrixXd::Identity(3,3);

    return H;
}

void EKF::prediction()
{
    std::cout << "prediction" << std::endl;

    //calcul de la période pour la prédiction
    ros::Duration duree = ros::Time::now() - m_temps;
    double period = duree.toSec();

    Vector3d cmdVel = m_poseSub.getVel();

    //calcul de la position du robot pour l'instant n+1
    m_xPredicted(0) = m_xMean(0) + period*(cos(m_xMean(2))*cmdVel(0)-sin(m_xMean(2))*cmdVel(1));
    m_xPredicted(1) = m_xMean(1) + period*(sin(m_xMean(2))*cmdVel(0)+cos(m_xMean(2))*cmdVel(1));
    m_xPredicted(2) = m_xMean(2) + period*cmdVel(2);

    m_xMean.block(0,0,3,1) = m_xPredicted.block(0,0,3,1);

    MatrixXd Fx = MatrixXd::Identity(m_P.rows(),m_P.cols());
    Fx(0,2) =  cmdVel(0)*cos(m_xMean(2))*period;
    Fx(1,2) = -cmdVel(1)*sin(m_xMean(2))*period;

    //mise à jour de m_P
    m_P_prev = Fx*m_P*(Fx.transpose());
    //mise à jour du temps
    m_temps = ros::Time::now();
}

void EKF::correction(geometry_msgs::Pose2D p, int i)
{
    std::cout << "correction\n" << std::endl;

    int size = m_P.rows();

    //calcul de z
    VectorXd mVect(3);
    mVect(0) = p.x;
    mVect(1) = p.y;
    mVect(2) = p.theta;

    Vector3d z = mVect - m_xMean.block(i,0,3,1);

    //calcul de H
    MatrixXd H = buildH2(p,size,i);
    //std::cout << "H = \n" << H << "\n" <<  std::endl;

    //calcul de R
    MatrixXd R(3,3);
    R.setZero();
    R(0,0) = 0.1;
    R(1,1) = 0.1;
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
    if((std::abs(z(0)) < 0.5) && (std::abs(z(1)) < 0.5) && (std::abs(z(2)) < 0.34))
    {
        m_xMean = m_xMean + K*z;
    }

    //mise à jour de la matrice m_P
    MatrixXd tmp = MatrixXd::Identity(Pm.rows(),Pm.cols()) - K*H;
    Pm = tmp*Pm;
    //std::cout << "Pm = \n" << Pm << "\n" <<  std::endl;
    updateP(Pm, i);
}

void EKF::printAreas()
{
    for (int i = 0; i < m_areas.size(); ++i)
    {
        std::cout << "machine in area " << m_areas[i] << std::endl;
    }
}

VectorXd EKF::getXmean()
{
    return m_xMean;
}

VectorXd EKF::getXpredicted()
{
    return m_xPredicted;
}

std::vector<int> EKF::getAreas()
{
    return m_areas;
}

std::vector<geometry_msgs::Pose2D> EKF::getTabMachines()
{
    return m_mps;
}
