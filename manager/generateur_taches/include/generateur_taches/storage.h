/**
 * \file 		storage.h
 * \class		Storage
 * \brief		classe représentant un produit stocké
 * \author		Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date		2015-04-01
 * \copyright	PyroTeam, Polytech-Lille
 */

#ifndef STORAGE_H
#define STORAGE_H

class Storage{

public:

	Storage(int product=0,int debut=0,int fin=0,int id=0);

	void setProduct(int prod){m_product=prod;}
	int getProduct(){return m_product;}
	void setBeginningDelivery(int deb){m_beginningDelivery=deb;}
	int getBeginningDelivery(){return m_beginningDelivery;}
	void setEndDelivery(int fin){m_endDelivery=fin;}
	int getEndDelivery(){return m_endDelivery;}
	void setId(int id){m_id=id;}
	int getId(){return m_id;}
  
private:

	int m_product; //si product = 10 alors il n'y aucun produit en stock
	int m_beginningDelivery;
	int m_endDelivery;
	int m_id;

};

#endif
