#include <vector>
#include <string>
#include "product.h"
#include <ros/ros.h>
#include <iostream>

using namespace std;

int Product::getRingColors(int i)
{
	int compteur = 0; 
	if(!m_ringColors.empty() && i>=0 && i<m_ringColors.size())
	{
		vector<uint8_t>::iterator it = m_ringColors.begin();
		while(compteur!=i)
		{
			it++;
			compteur++;
		}
	return (int)*it;
	}
	else
	{
		ROS_ERROR("Bad index vector access after line %d in line %s",__LINE__,__FILE__);
		return -1;
	} 
}
