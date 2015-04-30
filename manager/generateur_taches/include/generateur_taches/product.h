/**
 * \file 		product.h
 * \class		Product
 * \brief		classe représentant un produit
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */
 
#ifndef PRODUCT_H
#define PRODUCT_H

#include <vector>

class Product{

public:

	Product(int nbrRing,std::vector<int> parameters):m_parameters(parameters){
		m_nbrRing=nbrRing;
	}
  
	void setNbrRing(int nbrRing){m_nbrRing=nbrRing;}
	int getNbrRing(){return m_nbrRing;}
	void setParameters(std::vector<int> parameters){m_parameters=parameters;}
	std::vector<int> getParameters(){return m_parameters;}
	 
	int getParam(int i);
        	  
private:

	int m_nbrRing;
	std::vector<int> m_parameters;
  
};
#endif
