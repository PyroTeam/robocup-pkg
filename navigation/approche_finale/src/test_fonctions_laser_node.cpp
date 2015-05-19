#include <ros/ros.h>
#include <unistd.h>
#include "laserScan.h"
#include "Point.h"
#include "fa_utils.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"test_fonctions_laser");
	ros::NodeHandle n;
	ros::Time::init();
	ros::Rate loop_rate(1000);
	laserScan ls;
	loop_rate.sleep();
	geometry_msgs::Twist msg;
	std::vector<float> positionY(100);
	std::vector<float> gradient(100);
	int maxPoints=0;
	int nearby=0;
	float d=0;
	int j=0;
	float pos=0;
	ROS_INFO("Waiting a complete laserscan");
	while(ros::ok())
	{
	if(ls.getRanges().size() == 513)
	{
		maxPoints = ls.maxNumberPoints();
		nearby = ls.nearestSegment();
			
		//Segment s = ls.buildSegment(nearby);
		/*if(j < positionY.size())
		{
			if(pos > 0)
			{
				positionY[j] = ls.positionY(nearby,d);
				j++;
			}
		}
		else
		{				
			if(pos>0)
			{
				//s = ls.buildSegment(maxPoints);
				for(int i=0;i<positionY.size()-1;i++)
				{
					positionY[i] = positionY[i+1];
					gradient[i] = gradient[i+1];
				}
				positionY[positionY.size()-1] = pos;
				gradient[gradient.size()-1] = s.getGradient();
			}
		}
		ROS_INFO("le laser se situe a %f m du bord gauche de l'objet le plus proche",moy(positionY));
		ROS_INFO("gradient = %f",moy(gradient));*/
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
} 

