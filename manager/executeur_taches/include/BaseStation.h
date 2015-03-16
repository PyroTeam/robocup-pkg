#ifndef BASESTATION_H
#define BASESTATION_H

#include "Machine.h"

class BaseStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_baseRouge;
      int m_baseNoir;
      int m_baseGris;
      
  public:
    
      BaseStation();
      
      virtual ~BaseStation();
      
      virtual void FonctionVirtuelle();
      
       /* Méthodes */   
       int getBaseRouge();
       int getBaseNoir();      
       int getBaseGris();
       void majRouge(int nbRouge);
       void majNoir(int nbNoir);
       void majGris(int nbGris);
};

#endif