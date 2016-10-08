#include "Ekf.h"
#include "ros/ros.h"

using namespace Eigen;

EKF::EKF()
{
    m_state.conservativeResize(3);
    m_state.setZero();

    m_predictedState.conservativeResize(3);
    m_predictedState.setZero();

    m_P.conservativeResize(3,3);
    m_P.setZero();

    m_predictedP.conservativeResize(3,3);
    m_predictedP.setZero();

    m_time = ros::Time::now();
    m_initTime = ros::Time::now();

    m_begin = false;
}

// false while the subscriber does not give us a proper initial position
bool EKF::initOdom()
{
    Vector3d pose = m_poseSub.getPoseAsVector();

    if (std::abs(pose.sum()) > 1.0)
    {
        if (m_begin == false)
        {
            m_state(0) = pose(0);
            m_state(1) = pose(1);
            m_state(2) = pose(2);

            m_predictedState = m_state;

            m_begin = true;
        }
    }

    return m_begin;
}

void EKF::addMachine(const Vector3d &machine)
{
    //on redimensionne m_state et m_P pour accueillir la nouvelle machines
    m_state.conservativeResize(m_state.rows() + 3);
    m_predictedState.conservativeResize(m_state.rows());
    m_P.conservativeResize(m_P.rows()+3,m_P.cols()+3);
    m_predictedP.conservativeResize(m_P.rows(),m_P.cols());

    //on remplit avec les coordonnées de la nouvelle machine
    m_state.block(m_state.rows()-3,0,3,1) = machine;
    m_predictedState.block(m_state.rows()-3,0,3,1) = machine;

    std::cout << "machine ajoutée : " << m_state.block(m_state.rows()-3,0,3,1) << std::endl;

    //calcul de tous les PLi
    //initialisation des PLi à 0
    m_P.block(m_P.rows()-3,            0,          3, m_P.cols()).setZero();
    m_P.block(           0, m_P.cols()-3, m_P.rows(),          3).setZero();
    m_predictedP.block(m_predictedP.rows()-3,            0,          3, m_predictedP.cols()).setZero();
    m_predictedP.block(           0, m_predictedP.cols()-3, m_predictedP.rows(),          3).setZero();

    for (int j = 0; j < m_state.rows(); j = j + 3)
    {
        //position de la nouvelle machines par rapport au robot
        //et à toutes les autres
        double x     = m_state(m_state.rows()-3) - m_state(j  );
        double y     = m_state(m_state.rows()-2) - m_state(j+1);
        double theta = m_state(m_state.rows()-1) - m_state(j+2);

        m_P(m_P.rows()-3, j  ) = x;
        m_P(m_P.rows()-2, j+1) = y;
        m_P(m_P.rows()-1, j+2) = theta;

        m_predictedP(m_predictedP.rows()-3, j  ) = x;
        m_predictedP(m_predictedP.rows()-2, j+1) = y;
        m_predictedP(m_predictedP.rows()-1, j+2) = theta;

        m_P(j  , m_P.cols()-3) = x;
        m_P(j+1, m_P.cols()-2) = y;
        m_P(j+2, m_P.cols()-1) = theta;

        m_predictedP(j  , m_predictedP.cols()-3) = x;
        m_predictedP(j+1, m_predictedP.cols()-2) = y;
        m_predictedP(j+2, m_predictedP.cols()-1) = theta;

        m_P(j  , j  ) = m_state(j  ) - m_state(0);
        m_P(j+1, j+1) = m_state(j+1) - m_state(1);
        m_P(j+2, j+2) = m_state(j+2) - m_state(2);

        m_predictedP(j  , j  ) = m_state(j  ) - m_state(0);
        m_predictedP(j+1, j+1) = m_state(j+1) - m_state(1);
        m_predictedP(j+2, j+2) = m_state(j+2) - m_state(2);
    }
}

int EKF::checkStateVector(const Vector3d &machine)
{
    ROS_INFO_STREAM("State vector : \n" << m_state);
    int areaMachine = common_utils::getArea(machine);

    // 1 robot + 11 machines = 12 * 3 = 36
    if(m_state.rows() > 36) return -1;

    if (m_state.rows() > 3)
    {
        for (int i = 3; i < m_state.rows(); i=i+3)
        {
            if (areaMachine == common_utils::getArea(m_state.block(i,0,3,1)))
            {
                return i;
            }
        }
    }

    return 0;
}

MatrixXd EKF::buildH2(int size, int i)
{
    MatrixXd H(3,size);
    H.setZero();
    H.block(0,0,3,3) = MatrixXd::Identity(3,3);
    H.block(0,i,3,3) = MatrixXd::Identity(3,3);

    return H;
}

void EKF::predict()
{
    // calcul de la période pour la prédiction
    ros::Duration duree = ros::Time::now() - m_time;
    double period = duree.toSec();

    // récupération des commandes de vitesses
    Vector3d cmdVel = m_poseSub.getVel();

    // calcul de la position du robot pour l'instant n+1 à partir de l'instant n
    // x(t+1) = f(x(t), u(t))
    //        = x(t) + dx/dt*dt ( u(t) = dx/dt = (x(t+1)-x(t))/dt )

    double angle = m_predictedState(2);
    // Changement de repère des vitesses dans le repère odom
    double dVx = cos(angle)*cmdVel(0) - sin(angle)*cmdVel(1);
    double dVy = sin(angle)*cmdVel(0) + cos(angle)*cmdVel(1);

    // PREDICTION DE L'ETAT
    m_predictedState(0) = m_predictedState(0) +       dVx*period;
    m_predictedState(1) = m_predictedState(1) +       dVy*period;
    m_predictedState(2) = m_predictedState(2) + cmdVel(2)*period;

    // F matrice de transition. Elle est donnée par la Jacobienne de f

    //     [ dx/dx dx/dy dx/da ]   [   1     0   dx/da ]
    // F = [ dy/dx dy/dy dy/da ] = [   0     1   dy/da ]
    //     [ da/dx da/dy da/da ]   [   0     0     1   ]

    MatrixXd F = MatrixXd::Identity(m_P.rows(),m_P.cols());
    F(0,2) =  cos(angle)*cmdVel(0)*period;
    F(1,2) = -sin(angle)*cmdVel(1)*period;
    //std::cout << "F = \n" << F << "\n" <<  std::endl;

    // PREDICTION DE L'INCERTITUDE
    // (à déterminer covariance Q liée au bruit du système)
    m_predictedP = F*m_predictedP*(F.transpose());

    //mise à jour du temps
    m_time = ros::Time::now();
}

void EKF::run()
{
    // Phase de prédiction
    this->predict();

    // Si on détecte des machines
    if (m_landmarksSub.getMachines().size() != 0)
    {
        ROS_INFO("Update phase");
        // Phase de mise à jour
        this->correct();
    }
}

void EKF::correctOnce(int i)
{
    int size = m_P.rows();

    Vector3d seenMachine = m_landmarksSub.getMachine(i);
    int indexOfMachineInStateVector = checkStateVector(seenMachine);

    if (indexOfMachineInStateVector == 0)
    {
        this->addMachine(seenMachine);
    }
    else if (indexOfMachineInStateVector == -1)
    {
        // calcul de l'innovation
        // --> erreur entre la machine vue et la machine déjà détectée
        Vector3d z = seenMachine - m_state.block(indexOfMachineInStateVector,0,3,1);
        if (z(2) > M_PI_2)
        {
            z(2) -= M_PI;
        }
        else if (z(2) < -M_PI_2)
        {
            z(2) += M_PI;
        }
        //std::cout << "z = \n" << z << "\n" <<  std::endl;

        //calcul de la matrice de passage de l'espace d'état vers l'espace des mesures
        MatrixXd H = buildH2(size,indexOfMachineInStateVector);
        //std::cout << "H = \n" << H << "\n" <<  std::endl;

        //calcul de la matrice de covariance du bruit de mesure
        double noise = (3/100)*z.mean();
        MatrixXd R = noise*MatrixXd::Identity(3,3);
        //std::cout << "R = \n" << R << "\n" <<  std::endl;

        //calcul de la covariance de l'innovation résiduelle
        MatrixXd Z(3,3);
        Z.setZero();
        Z = H*m_predictedP*H.transpose() + R;
        //std::cout << "Z - R = \n" << H*Pm*H.transpose() << "\n" <<  std::endl;

        //calcul du gain de Kalman
        MatrixXd K;
        K.setZero();
        K = m_predictedP*H.transpose()*Z.inverse();
        //std::cout << "Gain = \n" << K << "\n" <<  std::endl;


        //mise à jour du vecteur d'état
        m_state = m_predictedState + K*z;
        //m_predictedState.block(0,0,3,1) = m_state.block(0,0,3,1);

        //mise à jour de la matrice P
        MatrixXd I = MatrixXd::Identity(m_predictedP.rows(),m_predictedP.cols());
        m_P = (I - K*H)*m_predictedP;
        //m_predictedP = m_P;
    }
}

void EKF::correct()
{
    for (int i = 0; i < m_landmarksSub.getMachines().size(); i++)
    {
        if (common_utils::getArea(m_landmarksSub.getMachine(i)) != 0)
        {
            this->correctOnce(i);
        }
    }
}

geometry_msgs::Pose2D EKF::getRobot()
{
    geometry_msgs::Pose2D tmp;
    tmp.x     = m_state(0);
    tmp.y     = m_state(1);
    tmp.theta = m_state(2);

    return tmp;
}

std::vector<geometry_msgs::Pose2D> EKF::getLandmarks()
{
    std::vector<geometry_msgs::Pose2D> vect;

    for (int i = 3; i < m_state.rows(); i = i+3)
    {
        geometry_msgs::Pose2D p;
        p.x     = m_state(i);
        p.y     = m_state(i+1);
        p.theta = m_state(i+2);
        vect.push_back(p);
    }
    return vect;
}
