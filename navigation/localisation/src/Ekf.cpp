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
    ROS_INFO_STREAM("State vector : " << m_state);
    int areaMachine = common_utils::getArea(machine);

    if (m_state.rows() > 3)
    {
        std::cout << "size de m_state :" << m_state.rows() << "\n" << std::endl;
        for (int i = 3; i < m_state.rows(); i=i+3)
        {
            ROS_ERROR("Zone %d VS Zone %d", areaMachine, common_utils::getArea(m_state.block(i,0,3,1)));
            ROS_WARN("(%f, %f) compared to (%f, %f)", machine(0), machine(1), m_state(i), m_state(i+1));

            if (areaMachine == common_utils::getArea(m_state.block(i,0,3,1)))
            {
                return i;
            }
            else
            {
                ROS_WARN("Machine not in the same zone");
            }
        }
    }
    else
    {
        ROS_WARN("Not yet machine in state vector");
    }

    return 0;
}

MatrixXd EKF::buildPm(int i)
{
    MatrixXd Pm(m_P.rows(),m_P.cols());
    Pm.setZero();

    Pm.block(0,0,3,3) = m_predictedP.block(0,0,3,3);
    Pm.block(i,i,3,3) = m_predictedP.block(i,i,3,3);
    Pm.block(0,i,3,3) = m_predictedP.block(0,i,3,3);
    Pm.block(i,0,3,3) = m_predictedP.block(i,0,3,3);

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
    m_predictedP.block(0,0,3,3) = Pm.block(0,0,3,3);
    m_predictedP.block(i,i,3,3) = Pm.block(i,i,3,3);
    m_predictedP.block(0,i,3,3) = Pm.block(0,i,3,3);
    m_predictedP.block(i,0,3,3) = Pm.block(i,0,3,3);
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
    //        = x(t) + dx/dt*dt (  dx/dt = (x(t+1)-x(t))/dt )

    double angle = m_predictedState(2);
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
    m_predictedP = F*m_P*(F.transpose());

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
    else
    {
        // calcul de l'erreur entre la machine vue et la machine déjà détectée
        Vector3d z = seenMachine - m_state.block(indexOfMachineInStateVector,0,3,1);
        //std::cout << "z = \n" << z << "\n" <<  std::endl;

        //calcul de H
        MatrixXd H = buildH2(size,indexOfMachineInStateVector);
        //std::cout << "H = \n" << H << "\n" <<  std::endl;

        //calcul de R (matrice de covariance du bruit)
        MatrixXd R = 0.1*MatrixXd::Identity(3,3);
        //std::cout << "R = \n" << R << "\n" <<  std::endl;

        //calcul de Pm
        //MatrixXd Pm = buildPm(indexOfMachineInStateVector);
        MatrixXd Pm = m_predictedP;
        //std::cout << "Pm =" << Pm << "\n" <<  std::endl;

        //calcul de Z
        MatrixXd Z(3,3);
        Z.setZero();
        Z = H*Pm*H.transpose() + R;
        //std::cout << "Z - R = \n" << H*Pm*H.transpose() << "\n" <<  std::endl;

        //calcul du gain de Kalman
        MatrixXd K;
        K.setZero();
        K = Pm*H.transpose()*Z.inverse();
        //std::cout << "Gain = \n" << K << "\n" <<  std::endl;


        //mise à jour du vecteur m_state
        m_state = m_predictedState + K*z;
        m_predictedState = m_state;

        //mise à jour de la matrice m_P
        MatrixXd I = MatrixXd::Identity(Pm.rows(),Pm.cols());
        Pm = (I - K*H)*Pm;
        //std::cout << "Pm maj = \n" << Pm << "\n" <<  std::endl;
        updateP(Pm, indexOfMachineInStateVector);
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
    tmp.x     = m_predictedState(0);
    tmp.y     = m_predictedState(1);
    tmp.theta = m_predictedState(2);

    return tmp;
}

std::vector<geometry_msgs::Pose2D> EKF::getLandmarks()
{
    std::vector<geometry_msgs::Pose2D> vect;

    for (int i = 3; i < m_predictedState.rows(); i = i+3)
    {
        geometry_msgs::Pose2D p;
        p.x     = m_predictedState(i);
        p.y     = m_predictedState(i+1);
        p.theta = m_predictedState(i+2);
        vect.push_back(p);
    }
    return vect;
}
