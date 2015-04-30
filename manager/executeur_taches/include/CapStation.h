#ifndef CAPSTATION_H
#define CAPSTATION_H

#include "Machine.h"

using namespace manager_msg;

class CapStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_blackCap;
      int m_greyCap;
      int m_stockID[3];
      int m_capID[3];
      
  public:
    
      CapStation();
      
      virtual ~CapStation();
      
      virtual void FonctionVirtuelle();
      
      /* Méthodes */
      int getGreyCap();
      int getBlackCap();  
      int getStockage(int i);
      void majStockID(int i, int val); 
      void majBlack(int nbNoir);
      void majGrey(int nbGris);
      void put_cap(int color, int n_robot, int n_order, int machine);
      void take_cap(int color, int n_robot, int n_order, int machine);
      void uncap(int color, int n_robot, int n_order,int machine);
      void stock(int id, int n_robot, int n_order,int machine);
      void destock(int id, int n_robot, int n_order,int machine);
};

#endif

