#include "Map.hpp"


std::vector<geometry_msgs::Pose2D> tab;


/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_gen");
    ros::NodeHandle n;
    ros::Subscriber sub_poses_machine    = n.subscribe("/machines", 1000, Poses_Machine_Callback);
    ros::Publisher Map_Pub = n.advertise<nav_msgs::OccupancyGrid>("/grid", 1000);
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
    	ROS_INFO("Machine %d : %f/%f",z,tab[z].x,tab[z].y);
		Get_One_Point_Of_The_Rectangle(tab[z].x, xA, tab[z].y, yA, tab[z].theta, 0.275, 0.450);
    	ROS_INFO("%f/%f",xA,yA);
		Set_Machines_In_Map(100, tab[z].theta, xA, yA, Map, 0.275, 0.450);
		// Map.data[1470+floor(xA/100)+floor(yA/100)*140]=100;
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
    	tab[i].x = tab[i].x;
    	tab[i].y = tab[i].y;
	}
   
}

void Create_Empty_Map(nav_msgs::OccupancyGrid &Map)
{
	Map.info.origin.position.x=-7.000;
	Map.info.origin.position.y=-1.000;
	Map.info.origin.position.z=0;
	Map.info.origin.orientation.x=0;
	Map.info.origin.orientation.y=0;
	Map.info.origin.orientation.z=0;
	Map.info.origin.orientation.w=1;
	Map.info.map_load_time=ros::Time::now();
	Map.info.resolution=0.0100;
	Map.info.width=1400;
	Map.info.height=800;
	Map.data.assign(Map.info.width*Map.info.height, 0);
}

void Set_Wall(nav_msgs::OccupancyGrid &Map)
{

	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{	
			if ((i<130)||(i>670)) Map.data[j+i*Map.info.width]=100;
			if ((j<130)||(j>1270)) Map.data[j+i*Map.info.width]=100;
		}
	}
	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{
			if ((i<100)||(i>700)) Map.data[j+i*Map.info.width]=50;
			if ((j<100)||(j>1300)) Map.data[j+i*Map.info.width]=50;
		}
	}

	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{
			if ((i<100)&&(j<400)) Map.data[j+i*Map.info.width]=0;
			if ((i<100)&&(j>1000)) Map.data[j+i*Map.info.width]=0;
		}
	}
	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{
			if (((i>90)&&(i<130))&&((j>120)&&(j<400))) Map.data[j+i*Map.info.width]=0;
			if (((i>90)&&(i<130))&&((j>1000)&&(j<1280))) Map.data[j+i*Map.info.width]=0;
		}
	}
	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{
			if ((i<130)&&((j>370)&&(j<410))) Map.data[j+i*Map.info.width]=100;
			if ((i<130)&&((j>1000)&&(j<1040))) Map.data[j+i*Map.info.width]=100;
		}
	}
	for (int i=0;i<Map.info.height;i++)
	{
		for (int j=0;j<Map.info.width;j++)
		{
			if (((i>90)&&(i<130))&&((j>90)&&(j<310))) Map.data[j+i*Map.info.width]=100;
			if (((i>90)&&(i<130))&&((j>1090)&&(j<1310))) Map.data[j+i*Map.info.width]=100;
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

void Set_Machines_In_Map(float rank, float theta, float xA, float yA, nav_msgs::OccupancyGrid &Map, float largeur, float longueur)
{
	float res = Map.info.resolution;
	int wid = Map.info.width;
	float xO = Map.info.origin.position.x;
	float yO = Map.info.origin.position.y;

	float projectionLongueur = fabs(longueur * cos(theta)) + fabs(largeur * sin(theta));
	float projectionLargueur = fabs(largeur * cos(theta)) + fabs(longueur * sin(theta));
	ROS_INFO("Longueur : %f | Largeur : %f",projectionLongueur, projectionLargueur);

	int nbWidthCells = ceil(longueur/res);
	int nbHeightCells = ceil(largeur/res);
	ROS_INFO("Nb Cells Longueur : %d | Nb Cells Largeur : %d",nbWidthCells, nbHeightCells);

	int cellOrigin = (int)floor((xA-xO)/res)+(int)floor((yA-yO)/res*wid);

	for (int w = 0; w < nbWidthCells; ++w)
	{
		for (int h = 0; h < nbHeightCells; ++h)
		{
			float xN = xA + fabs(res * w * cos(theta)) + fabs(res * h * sin(theta));
			float yN = yA + fabs(res * h * cos(theta)) + fabs(res * w * sin(theta));

			int cell = round((xN-xO)/res)+round((yN-yO)/res)*wid;;
			ROS_INFO("%d/%d _ %d",w, h, cell);
			ROS_INFO("%f/%f | %f",xN, yN, theta);
			Map.data[cell]=rank;
		}
	}
}