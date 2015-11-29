/**
 * \file 			order.h
 * \class			Order
 * \brief			classe représentant un ordre spécifié par la RefBox
 * \author			Smagghe Cyril (cyril.smagghe@polytech-lille.net)
 * \date			2015-04-01
 * \copyright		PyroTeam, Polytech-Lille
 */

#ifndef ORDER_H
#define ORDER_H

#include <vector>
#include "product.h"

class Order{

public:

	Order(Product product,int beginningDelivery,int endDelivery,int quantity,bool processed):m_product(product){    
	    m_beginningDelivery=beginningDelivery;
    	m_endDelivery=endDelivery;
    	m_quantity=quantity;
    	m_processed=processed;
    }
  
	void setProduct(Product prod){m_product=prod;}
	Product getProduct(){return m_product;}
	void setBeginningDelivery(int deb){m_beginningDelivery=deb;}
	int getBeginningDelivery(){return m_beginningDelivery;}
	void setEndDelivery(int fin){m_endDelivery=fin;}
	int getEndDelivery(){return m_endDelivery;}
	void setQuantity(int quan){m_quantity=quan;}
	int getQuantity(){return m_quantity;}
	void setProcessed(bool processed){m_processed=processed;}
	bool getProcessed(){return m_processed;}
	  
private:

	Product m_product;
	int m_beginningDelivery;
	int m_endDelivery;
	int m_quantity;
	bool m_processed;

};

#endif
