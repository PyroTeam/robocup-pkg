#include "ros/ros.h"
#include "std_msgs/String.h"

#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"
#include "geometry_msgs/Pose2D.h"


using namespace std;

int main(int argc, char **argv) {
/* TESTS */ 
    BaseStation basestation;// = new BaseStation;
    RingStation ringstation;// = new RingStation;
    CapStation capstation;// = new capStation;
    DeliveryStation deliverystation;// = new DeliveryStation;
    //std::cout << basestation.type;
    std::cout << "Hello Sandra!!" << std::endl;
    std::cout << "Test avant : " << basestation.getEntreeMachine().x <<" "<< basestation.getEntreeMachine().y << std::endl;
    
    geometry_msgs::Pose2D monPoint;
    monPoint.x = 3.2;
    monPoint.y = 4.5;
    monPoint.theta = 0.4;
    basestation.majEntree(monPoint);
    std::cout << "Test apres : " << basestation.getEntreeMachine().x <<" "<< basestation.getEntreeMachine().y << std::endl;
    
}