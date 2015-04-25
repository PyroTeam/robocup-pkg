#include "ReportingMachineSrvClient.h"

  ReportingMachineSrvClient::ReportingMachineSrvClient() {}
  ReportingMachineSrvClient::~ReportingMachineSrvClient(){}

bool ReportingMachineSrvClient::reporting(std::string r_name, std::string r_type, uint8_t r_zone){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<comm_msg::ReportMachine>("/refBoxComm/ReportMachine");
  comm_msg::ReportMachine srv;
  srv.request.name = r_name;
  srv.request.type = r_type;
  srv.request.zone = r_zone;
  if (client.call(srv))
  {
    ROS_INFO("State: No problem");
  }
  else
  {
    ROS_ERROR("Failed to call service ReportMachine");
    return true;
  }

  return false;

}

