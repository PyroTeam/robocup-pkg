#ifndef NAVIGATIONCLIENTACTION_H
#define NAVIGATIONCLIENTACTION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <deplacement_msg/MoveToPoseAction.h>


class NavigationClientAction{
public :
	NavigationClientAction();
	virtual  ~NavigationClientAction();
	int goToAPoint(geometry_msgs::Pose2D dest_point);
};
#endif
