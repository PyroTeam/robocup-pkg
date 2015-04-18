#ifndef BASESTATION_H
#define BASESTATION_H

#include "Machine.h"

using namespace manager_msg;

class BaseStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_redBase;
      int m_blackBase;
      int m_silverBase;
      
  public:
    
      BaseStation();
      
      virtual ~BaseStation();
      
      virtual void FonctionVirtuelle();
      
       /* Méthodes */   
       int getRedBase();
       int getBlackBase();      
       int getSilverBase();
       void majRed(int nbRouge);
       void majBlack(int nbNoir);
       void majSilver(int nbArgent);
       void take_base(int color,int n_robot,int n_order);
       void bring_base_rs(int color,int n_robot,int n_order,int machine);
};

#endif
