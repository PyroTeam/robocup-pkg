#include "ArTagClientSrv.h"

ArTagClienSrv::ArTagClienSrv() {}

ArTagClienSrv::~ArTagClienSrv() {}

int16_t ArTagClienSrv::askForId()
{
  	ros::NodeHandle n;
  	ros::ServiceClient client = n.serviceClient<trait_im_msg::artag>("/artag");
  	trait_im_msg::artag srv;
  	if (client.call(srv))
  	{
		m_id = srv.response.id;
		ROS_INFO("State: No problem, the id is %d", (int)m_id);
		return m_id;
  	}
  	else
  	{
		ROS_ERROR("Failed to call service artag");
		return -1;
  	}
}