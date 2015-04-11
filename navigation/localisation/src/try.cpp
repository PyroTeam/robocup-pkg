#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <random>
#include "ros/ros.h"

using namespace Eigen;

int main()
{
	MatrixXd P(12,12);
	P.setRandom();
	MatrixXd H(3,12);
	H.block(0,0,3,3) = MatrixXd::Identity(3,3);
	H.block(0,6,3,3) = MatrixXd::Identity(3,3);

	std::cout << P << "\n" << std::endl;
	std::cout << H << "\n" << std::endl;
	std::cout << H.transpose() << "\n" << std::endl;
	std::cout << H*P*H.transpose() << "\n" << std::endl;
}