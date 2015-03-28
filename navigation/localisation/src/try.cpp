#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

int main()
{
 	Matrix3d m;
	m.setZero();
	VectorXd before(3);
	VectorXd after(3);

	//translation
	m(1,2) = -0.2;
	//rotation
	m(0,0) =  cos(90*M_PI/180);
	m(0,1) = -sin(90*M_PI/180);
	m(1,0) =  sin(90*M_PI/180);
	m(1,1) =  cos(90*M_PI/180);

	m(2,2) = 1;

	//before << 0, 0, 1;
	before(0) = 1;
	before(1) = 1;
	before(2) = 0;

	std::cout << before << std::endl;

	after = m*before;

 	std::cout << m << std::endl;
 	std::cout << after << std::endl;
 	
}