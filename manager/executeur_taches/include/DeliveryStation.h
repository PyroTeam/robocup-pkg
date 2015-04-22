#ifndef DELIVERYSTATION_H
#define DELIVERYSTATION_H

#include "Machine.h"

using namespace manager_msg;

class DeliveryStation : public Machine{
  private:
      
  public:
    
    DeliveryStation();
      
    virtual ~DeliveryStation();
    
    virtual void FonctionVirtuelle();

    void deliverToDS(int n_robot, int n_order); 
  
};

#endif