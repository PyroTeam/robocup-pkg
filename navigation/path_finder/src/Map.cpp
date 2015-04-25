#include "Map.hpp"


std::vector<geometry_msgs::Pose2D> tab;


/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::Subscriber sub_poses_machine    = n.subscribe("/machines", 1000, Poses_Machine_Callback);
    ros::Publisher Map_Pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);
    ROS_INFO("Ready to Generate the Map");
    float xA,yA;
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
    nav_msgs::OccupancyGrid Map;
    Create_Empty_Map(Map);
    Set_Wall(Map);
    for (int z=0;z<tab.size();z++)
    {
		Get_One_Point_Of_The_Rectangle(tab[z].x, xA, tab[z].y, yA, tab[z].theta, 275, 450);
		Set_Machines_In_Map(100, tab[z].theta, xA, yA, Map);
		Map.data[1470+floor(xA/100)+floor(yA/100)*140]=100;
  	}
 	Map_Pub.publish(Map);
 	ros::spinOnce();
	loop_rate.sleep();
 	}
    return 0;
}

void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines)
{
	tab=machines->landmarks;
	for (int i=0; i<tab.size(); i++)
    {
    	tab[i].x = tab[i].x*1000;
    	tab[i].y = tab[i].y*1000;
	}
   
}

void Create_Empty_Map(nav_msgs::OccupancyGrid &Map)
{
	Map.info.origin.position.x=-7000;
	Map.info.origin.position.y=-1000;
	Map.info.origin.position.z=0;
	Map.info.origin.orientation.x=0;
	Map.info.origin.orientation.y=0;
	Map.info.origin.orientation.z=0;
	Map.info.origin.orientation.w=1;
	Map.info.map_load_time=ros::Time::now();
	Map.info.resolution=100;
	Map.info.width=140;
	Map.info.height=80;
	Map.data.assign(Map.info.width*Map.info.height, 0);
}

void Set_Wall(nav_msgs::OccupancyGrid &Map)
{

	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{	
			if ((i<13)||(i>67)) Map.data[j+i*140]=100;
			if ((j<13)||(j>127)) Map.data[j+i*140]=100;
		}
	}
	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{
			if ((i<10)||(i>70)) Map.data[j+i*140]=50;
			if ((j<10)||(j>130)) Map.data[j+i*140]=50;
		}
	}

	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{
			if ((i<10)&&(j<40)) Map.data[j+i*140]=0;
			if ((i<10)&&(j>100)) Map.data[j+i*140]=0;
		}
	}
	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{
			if (((i>9)&&(i<13))&&((j>12)&&(j<40))) Map.data[j+i*140]=0;
			if (((i>9)&&(i<13))&&((j>100)&&(j<128))) Map.data[j+i*140]=0;
		}
	}
	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{
			if ((i<13)&&((j>37)&&(j<41))) Map.data[j+i*140]=100;
			if ((i<13)&&((j>100)&&(j<104))) Map.data[j+i*140]=100;
		}
	}
	for (int i=0;i<80;i++)
	{
		for (int j=0;j<140;j++)
		{
			if (((i>9)&&(i<13))&&((j>9)&&(j<31))) Map.data[j+i*140]=100;
			if (((i>9)&&(i<13))&&((j>109)&&(j<131))) Map.data[j+i*140]=100;
		}
	}

}

void Get_One_Point_Of_The_Rectangle(float x, float &xA, float y, float &yA, float theta, float largeur, float longueur)
{
	float x1,y1;
	x1 = x - cos(theta)*longueur;
	y1 = y - sin(theta)*longueur;

	xA = x1 + sin(M_PI_2-theta)*largeur;
	yA = y1 - cos(M_PI_2-theta)*largeur;
}

void Set_Machines_In_Map(float rank, float theta, float xA, float yA, nav_msgs::OccupancyGrid &Map)
{
	float x=xA;
	float y=yA;
		if (theta<=M_PI_2)
		{
			for (int j=0;j<10;j++)
			{
				for (int i=0;i<18;i++)
				{
					Map.data[1470+floor(x/rank)+floor(y/rank)*140]=100;
					x = x + cos(theta)*50;
					y = y + sin(theta)*50;
				}
				x = xA - j*cos(M_PI_2-theta)*50;
				y = yA + j*sin(M_PI_2-theta)*50;
			}
			
		}

		if (theta>M_PI_2)
		{
			for (int j=0;j<10;j++)
			{
				for (int i=0;i<18;i++)
				{
					Map.data[1470+floor(x/rank)+floor(y/rank)*140]=100;
					x = x - sin(theta-M_PI_2)*50;
					y = y + cos(theta-M_PI_2)*50;
				}
				x = xA - j*cos(theta-M_PI_2)*50;
				y = yA - j*sin(theta-M_PI_2)*50;
			}
			
		}
}