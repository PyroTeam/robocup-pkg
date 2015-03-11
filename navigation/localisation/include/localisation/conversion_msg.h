#include "Point.h"
#include "Modele.h"
#include "geometry_msgs/Pose2D.h"
#include "deplacement_msg/Machines.h"

geometry_msgs::Pose2D convertMachineToPose2D(Machine m);

deplacement_msg::Machines fillTabFrom(std::list<Machine> listOfMachines);