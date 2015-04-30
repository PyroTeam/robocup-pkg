#include <vector>
#include <string>
#include "product.h"

#include <iostream>

using namespace std;

int Product::getParam(int i){
        int compteur = 0; 
        vector<int>::iterator it = m_parameters.begin();
        if(compteur==i)
        {
        	return *it;
        }
        else 
        {
        	while(compteur!=i)
    		{
                it++;
                compteur++;
			}
		}
        return *it; 
}
