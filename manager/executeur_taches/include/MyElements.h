#ifndef MYELEMENTS_H
#define MYELEMENTS_H

#include "Machine.h"
#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"

class MyElements{
  private:
    
      /* Variables d'instance*/
      BaseStation bs;
      RingStation rs1; 
      RingStation rs2;
      CapStation cs1; 
      CapStation cs2;
      DeliveryStation ds; 
      
  public:
    
      MyElements();
      
      virtual ~MyElements();

      /* Méthodes */ 
      BaseStation getBS();
      RingStation getRS1();
      RingStation getRS2();
      CapStation  getCS1();
      CapStation  getCS2();
      DeliveryStation getDS();

};

#endif