#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <iostream>

int main(int argc, char **argv)
{
    Eigen::Matrix3f Mbr;
    Mbr << cos(-1.24),-sin(-1.24),-3.2,sin(1.24),cos(1.24),4.44,0,0,1;

    Eigen::Matrix3f Mbm;
    Mbm << cos(0.31),-sin(0.31),-3,sin(0.31),cos(0.31),3.75,0,0,1;

    Eigen::Matrix3f Mmr;
    Mmr = Mbm.inverse()*Mbr;

    std::cout<<Mmr<<std::endl;
    std::cout<<Mmr(1,2)<<std::endl;

	return 0;
}
