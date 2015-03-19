#ifndef BASESTATION_H
#define BASESTATION_H

#include "Machine.h"

class BaseStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_baseRouge;
      int m_baseNoir;
      int m_baseArgent;
      
  public:
    
      BaseStation();
      
      virtual ~BaseStation();
      
      virtual void FonctionVirtuelle();
      
       /* Méthodes */   
       int getBaseRouge();
       int getBaseNoir();      
       int getBaseArgent();
       void majRouge(int nbRouge);
       void majNoir(int nbNoir);
       void majArgent(int nbArgent);
};

#endif