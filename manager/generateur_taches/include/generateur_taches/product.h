/**
 * \file 		product.h
 * \class		Product
 * \brief		classe représentant un produit
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright   2016, Association de Robotique de Polytech Lille All rights reserved
 */
 
#ifndef PRODUCT_H
#define PRODUCT_H

#include <vector>

class Product{

public:

	Product(int complexity,int baseColor, std::vector<uint8_t> ringColors, int capColor):m_ringColors(ringColors)
	{
		m_complexity=complexity;
		m_baseColor=baseColor;
		m_capColor=capColor;
	}
  
	void setComplexity(int complexity){m_complexity=complexity;}
	void setBaseColor(int baseColor){m_baseColor = baseColor;}
	void setRingColors(std::vector<uint8_t> ringColors){m_ringColors=ringColors;}
	void setCapColor(int capColor){m_capColor = capColor;}
	
	int getComplexity(){return m_complexity;} 
	int getBaseColor(){return m_baseColor;}
	int getRingColors(int i);
	int getCapColor(){return m_capColor;}
        	  
private:

	int m_complexity;
	int m_baseColor;
	std::vector<uint8_t> m_ringColors;
	int m_capColor;  
};
#endif
