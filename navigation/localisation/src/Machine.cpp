#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "Machine.h"
#include "math_functions.h"
#include "common_utils/Zone.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace common_utils;

Machine::Machine() : m_xSum(0.0),m_ySum(0.0),m_nbActu(0.0), m_lastError(std::numeric_limits<int>::max())
{

};

Machine::~Machine()
{

}

geometry_msgs::Pose2D Machine::reversePose()
{
    geometry_msgs::Pose2D tmp, center = pose();
    tmp.x = -center.x;
    tmp.y = center.y;
    tmp.theta = geometry_utils::normalizeAngle(M_PI - center.theta);

    return tmp;
}

deplacement_msg::Machine Machine::msg()
{
    deplacement_msg::Machine tmp;
    tmp.pose = pose();
    tmp.zone = zone();
    tmp.orientationOk = checkOrientation();
    tmp.idIn = idIn();
    tmp.idOut = idOut();
    return tmp;
}

int Machine::getNbActu()
{
    return m_nbActu;
}

double Machine::getLastError()
{
    return m_lastError;
}

void Machine::theta(double theta)
{
    geometry_msgs::Pose2D tmp = pose();
    tmp.theta = theta;
    pose(tmp);
}

void Machine::update(const geometry_msgs::Pose2D &p)
{
    geometry_msgs::Pose2D center = pose();

    m_xSum += p.x;
    m_ySum += p.y;
    m_nbActu++;

    center.x     = m_xSum/double(m_nbActu);
    center.y     = m_ySum/double(m_nbActu);

    double tmp = geometry_utils::normalizeAngle(p.theta);
    double diff = std::abs(center.theta - tmp);
    // Si l'écart est grand cela signifie que la machine est vue avec le mauvais angle par rapport à celui enregistré
    // cela est dû à la transformation TF du repère tower_camera_link vers map
    if (diff > M_PI_2)
    {
        if (tmp > 0)
        {
            tmp -= M_PI;
        }
        else
        {
            tmp += M_PI;
        }
    }
    center.theta = geometry_utils::normalizeAngle((center.theta + tmp)/2);

    m_lastError = geometry_utils::distance(center, p) + std::abs(diff);

    pose(center);
}

void Machine::calculateCoordMachine(Segment s)
{
    double angle     = s.getAngle();
    double absMilieu = (s.getMax().x + s.getMin().x)/2;
    double ordMilieu = (s.getMax().y + s.getMin().y)/2;
    double sinus     = 0.35/2*sin(angle);
    double cosinus   = 0.35/2*cos(angle);

    geometry_msgs::Pose2D center;

    double d1 = (absMilieu - sinus)*(absMilieu - sinus) + (ordMilieu + cosinus)*(ordMilieu + cosinus);
    double d2 = (absMilieu + sinus)*(absMilieu + sinus) + (ordMilieu - cosinus)*(ordMilieu - cosinus);

    if (d1 > d2)
    {
        center.x = absMilieu - sinus;
        center.y = ordMilieu + cosinus;
        center.theta = angle;
    }
    else
    {
        center.x = absMilieu + sinus;
        center.y = ordMilieu - cosinus;
        center.theta = angle;
    }

    pose(center);
}

bool Machine::neverSeen()
{
    return m_nbActu == 0;
}

void Machine::switchSides()
{
    double angle = theta();

    if (angle < 0)
    {
        angle += M_PI;
    }
    else
    {
        angle -= M_PI;
    }

    theta(angle);

    setOrientation();
}

bool Machine::isInsideZone(int zone)
{
    // on crée une zone temporaire avec le numéro de zone donné
    Zone area(zone);

    // mps width is 0.35 m
    // si la machine est dans le rectangle inscrit à la zone, c'est bon
    if (area.isInside(pose(), 0.35))
    {
        return true;
    }
    else
    {
        return false;
    }
}
