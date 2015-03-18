#ifndef CAPSTATION_H
#define CAPSTATION_H

#include "Machine.h"

class CapStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_capNoir;
      int m_capGris;
      int m_stock[3];
      geometry_msgs::Pose2D m_stockage;
      
  public:
    
      CapStation();
      
      virtual ~CapStation();
      
      virtual void FonctionVirtuelle();
      
      /* Méthodes */
      int getCapGris();
      int getCapNoir();      
      void majNoir(int nbNoir);
      void majGris(int nbGris);
      //int stocker();

};

#endif