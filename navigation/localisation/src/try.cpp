#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <random>

using namespace Eigen;

int main()
{
	// construct a trivial random generator engine from a time-based seed:
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);

	std::normal_distribution<double> distribution (0.0,0.1);

	Vector3d tmp;
	tmp(0) = 1;
	tmp(1) = 1;
	tmp(2) = 1;

 	Vector3d xPredicted;
 	Vector3d cmdVelVect;

 	xPredicted.setZero();
 	cmdVelVect.setZero();

	cmdVelVect(0) = 2;
	cmdVelVect(1) = 2;
	cmdVelVect(2) = 0;

	Vector3d deltaX;

	deltaX = 0.01*cmdVelVect;

	xPredicted = tmp.topLeftCorner(3,1) + deltaX;

	//std::cout << cmdVelVect << "\n" << std::endl;
	std::cout << deltaX << "\n" << std::endl;
	std::cout << xPredicted << "\n" << std::endl;
	std::cout << tmp.topLeftCorner(3,1) << "\n" << std::endl;

/*
 	MatrixXd N(m.rows(), m.cols());
 	int l = 0;

 	//N matrice du bruit gaussien (on choisit un bruit de l'ordre du cm)
 	//matrice symétrique
 	for (int i = 0; i < N.rows(); ++i){
 		for (int j = 0; j < N.cols(); ++j){
    		double num = floor(distribution(generator)*100);
    		if (j >= i){
    			N(i,j) = num;
    		}
 			else {
 				N(i,j) = N(j,i);
 			}
 		}
 	}

 	int j = 0;
 	for (int i = 3; i < xMean.rows(); i=i+3){
 		if (after(0) == xMean(i) && after(1) == xMean(i+1)){
 			j = i;
 		}
 	}
 	*/
}