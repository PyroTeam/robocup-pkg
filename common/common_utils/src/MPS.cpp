#include "common_utils/MPS.h"
#include "common_utils/Zone.h"

namespace common_utils {

MPS::MPS() : m_orientationOk(false), m_zone(0)
{

}

MPS::MPS(const geometry_msgs::Pose2D &pose) : m_orientationOk(false)
{
    m_pose = pose;
    m_zone = getArea(pose);
}

MPS::~MPS()
{

}

void MPS::pose(const geometry_msgs::Pose2D &pose)
{
    m_pose = pose;
}

void MPS::zone(int area)
{
    m_zone = area;
}

void MPS::id(int ARTagId)
{
    // si c'est un ID d'entrée
    if (ARTagId%2 == 1)
    {
        m_idIn  = ARTagId;
        m_idOut = ARTagId + 1;
    }
    else
    {
        m_idIn  = ARTagId - 1;
        m_idOut = ARTagId;
    }
}

void MPS::setOrientation()
{
    m_orientationOk = true;
}

deplacement_msg::MPS Machine::msg()
{
    deplacement_msg::MPS tmp;

    tmp.pose = pose();
    tmp.zone = zone();
    tmp.orientationOk = checkOrientation();
    tmp.idIn = idIn();
    tmp.idOut = idOut();

    return tmp;
}

} // namespace common_utils
