#ifndef RINGTATION_H
#define RINGSTATION_H

#include "Machine.h"

class RingStation : public Machine{
  private:
    
      /* Variables d'instance*/
      int m_ringVert;
      int m_ringJaune;
      int m_ringBleu;
      int m_ringOrange;
      
  public:
      RingStation();
      
      virtual ~RingStation();
      
      virtual void FonctionVirtuelle();
      
      /* Méthodes */
      int getRingVert();
      int getRingJaune();  
      int getRingBleu();
      int getRingOrange(); 
      void majVert(int nbVert);
      void majJaune(int nbJaune);
      void majBleu(int nbBleu);
      void majOrange(int nbOrange);
};

#endif