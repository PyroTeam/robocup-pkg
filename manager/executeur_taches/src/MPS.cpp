#include "MPS.h"
#include "geometry_msgs/Pose2D.h"
#include "common_utils/types.h"

using namespace common_utils;

MPS::MPS()
{
    zone   = -1;
    isHere = false;
}

MPS::~MPS(){}

bool MPS::isDS()
{
    if(idIn == C_DS_IN || idOut == C_DS_OUT || idIn == M_DS_IN || idOut == M_DS_OUT)
    {
        return true;
    }
    return false;
}
