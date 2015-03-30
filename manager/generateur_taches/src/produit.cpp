#include <vector>
#include <string>
#include "produit.h"

#include <iostream>

using namespace std;

string Produit::get_param(int i){
        int compteur = 0; 
        vector<string>::iterator it=m_parametres.begin();
        if(compteur==i) return *it;
        
        else {while(compteur!=i){
                it++;
                compteur++;
                }}
        return *it; 
}
