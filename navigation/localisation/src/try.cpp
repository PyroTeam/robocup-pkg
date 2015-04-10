#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <random>

using namespace Eigen;

int main()
{
	VectorXd xMean(9);
	xMean(0) = 0.5;
	xMean(1) = 0.5;
	xMean(2) = 0;

	xMean(3) = 1;
	xMean(4) = 1;
	xMean(5) = 0;

	xMean(6) = 2;
	xMean(7) = 2;
	xMean(8) = 0;


	MatrixXd P(9,9);
	P.setRandom();

	P.block(P.rows() - 3, 0, 3, P.cols()).setZero();
	P.block(0, P.cols() - 3, P.rows(), 3).setZero();

	double x, y, theta;

	//remplissage avec les vecteurs de positionnement
	for (int i = 0; i < P.cols(); i = i + 3){
		for (int j = 0; j < xMean.rows(); j = j + 3){
			//dérivées partielles suivant x, y et theta
			x 	  = xMean(xMean.rows()-3) - xMean(j  );
			y 	  = xMean(xMean.rows()-2) - xMean(j+1);
			theta = xMean(xMean.rows()-1) - xMean(j+2);

			P(P.rows()-3, i  ) = x;
			P(P.rows()-2, i+1) = y;
			P(P.rows()-1, i+2) = theta;

			std::cout << P << "\n" << std::endl;
		}
	}
}