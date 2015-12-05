#include "Map.hpp"

#define MARGIN 0.3

std::vector<geometry_msgs::Pose2D> tab;

class Machine
{
public:
	Machine(int zone = 0):
	zone(zone),
	nbActu(0),
	xSum(0),
	ySum(0),
	thetaSum(0)
	{
		x=0;y=0;theta=0;draw=false;
	};
	~Machine(){};

	float x;
	float y;
	float theta;
	int zone;
	bool draw;

	int nbActu;
	float xSum;
	float ySum;
	float thetaSum;
	
};
std::vector<Machine> mps(24);



/*==========  Main  ==========*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_gen");
    ros::NodeHandle n;
    // ros::Subscriber sub_poses_machine    = n.subscribe("objectDetection/machines", 1000, Poses_Machine_Callback);
    ros::Subscriber sub_poses_machine    = n.subscribe("objectDetection/landmarks", 1000, Poses_Machine_Callback);
    ros::Publisher Map_Pub = n.advertise<nav_msgs::OccupancyGrid>("objectDetection/grid", 1000);
    ROS_INFO("Ready to Generate the Map");
    float xA,yA;
    ros::Rate loop_rate(1);

    // Empty map
    nav_msgs::OccupancyGrid Map;
    Create_Empty_Map(Map);

    // ROS Loop
    while(ros::ok())
    {
    	// // Draw MPS if needed (ABORTED TEMPORARILY)
   		// for(int i=0; i < mps.size(); ++i) {
   		// 	if(mps[i].draw) {
   		// 		// ERASE
   		// 		eraseZone(Map, mps[i].zone);

   		// 		// DRAW
	    // 		drawRect(Map, mps[i].x, mps[i].y, mps[i].theta,0.35,0.7,0.3);
	    // 		mps[i].draw = false;
   		// 	}
   		// }

    	// Redraw all map
    	Create_Empty_Map(Map);
   		Set_Wall(Map);
   		// Set_Forbidden_Zone(Map);
   		/*for(int i=0; i < mps.size(); ++i) {
			// DRAW
			if(mps[i].draw) {
	    		drawRect(Map, mps[i].x, mps[i].y, mps[i].theta,0.35,0.7,MARGIN);
	    		mps[i].draw = false;
	    	}
   		}*/

	 	Map_Pub.publish(Map);
	 	ros::spinOnce();
		loop_rate.sleep();
 	}

    return 0;
}

void Poses_Machine_Callback(const deplacement_msg::LandmarksConstPtr &machines)
{
	// Romain legacy
	tab=machines->landmarks;

	// Valentin
	// Remplissage du tableau de MPS
	for (int i=0; i< machines->landmarks.size(); i++)
    {
    	int zone = getZone(machines->landmarks[i].x, machines->landmarks[i].y);
    	if(zone==0)
    		continue;

    	mps[zone-1].xSum += machines->landmarks[i].x;
    	mps[zone-1].ySum += machines->landmarks[i].y;
    	mps[zone-1].thetaSum += machines->landmarks[i].theta;
    	mps[zone-1].nbActu += 1;

    	// Version moyenee
    	// mps[zone-1].x = mps[zone-1].xSum/mps[zone-1].nbActu;
    	// mps[zone-1].y = mps[zone-1].ySum/mps[zone-1].nbActu;
    	// mps[zone-1].theta = mps[zone-1].thetaSum/mps[zone-1].nbActu;
    	// mps[zone-1].draw = true;
    	// mps[zone-1].zone = zone;

    	// Version standard
    	mps[zone-1].x = machines->landmarks[i].x;
    	mps[zone-1].y = machines->landmarks[i].y;
    	mps[zone-1].theta = machines->landmarks[i].theta;
    	mps[zone-1].draw = true;
    	mps[zone-1].zone = zone;
	}   
}

void Create_Empty_Map(nav_msgs::OccupancyGrid &Map)
{
	Map.header.frame_id = "/odom";
	Map.info.origin.position.x=-7.000;
	Map.info.origin.position.y=-2.000;
	Map.info.origin.position.z=0;
	Map.info.origin.orientation.x=0;
	Map.info.origin.orientation.y=0;
	Map.info.origin.orientation.z=0;
	Map.info.origin.orientation.w=1;
	Map.info.map_load_time=ros::Time::now();
	Map.info.resolution=0.05;
	Map.info.width=14/Map.info.resolution;
	Map.info.height=9/Map.info.resolution;
	Map.data.assign(Map.info.width*Map.info.height, 0);
}

void Set_Wall(nav_msgs::OccupancyGrid &Map)
{
        /*drawRect(Map, -3, 2.5, 0, 1, 0.05, MARGIN);
        drawRect(Map, -3, 0.5, 0, 1, 0.05, MARGIN);

        drawRect(Map, -2.5, 0, 0, 0.05, 1, MARGIN);
        drawRect(Map, 0, 0, 0, 0.05, 2, MARGIN);
        drawRect(Map, 2.5, 0, 0, 0.05, 1, MARGIN);

        drawRect(Map, 3, 1.5, 0, 3, 0.05, MARGIN);
        drawRect(Map, 0, 3, 0, 0.05, 6, MARGIN);*/
}

void Set_Forbidden_Zone(nav_msgs::OccupancyGrid &Map)
{
	// Left Side
		// R1
    	drawRect(Map, -4, 5.25, 0, 1.5, 4, 0); 
    	// R2
    	drawRect(Map, -5.1, 3.75, 0, 1.5, 1.8, 0); 
    	// R3
    	drawRect(Map, -4.1, 2.25, 0, 1, 0.2, 0); 
    	// R4
    	drawRect(Map, -3, 2.25, 0, 1.5, 2, 0); 
    	// R5
    	drawRect(Map, -5.1, 0.75, 0, 1.5, 1.8, 0); 
    	// R6
    	drawRect(Map, -3, 0.75, 0, 0.7, 1, 0); 
    	// R7
    	drawRect(Map, -3.5, 1.25, 0, 0.5, 1, 0); 
    	// R8
    	drawRect(Map, -3.25, 0.75, 0, 0.5, 0.5, 0); 

    	// IN RIGHT
    	drawRect(Map, -(2.55+0.25), 0, 0, 1.5, 0.5, 0);     	
    	// IN LEFT
    	drawRect(Map, -5, -0.2, 0, 0.5, 2.5, 0);

	// Right side
		// R1
    	drawRect(Map, 4, 5.25, 0, 1.5, 4, 0); 
    	// R2
    	drawRect(Map, 5.1, 3.75, 0, 1.5, 1.8, 0); 
    	// R3
    	drawRect(Map, 4.1, 2.25, 0, 1, 0.2, 0); 
    	// R4
    	drawRect(Map, 3, 2.25, 0, 1.5, 2, 0); 
    	// R5
    	drawRect(Map, 5.1, 0.75, 0, 1.5, 1.8, 0); 
    	// R6
    	drawRect(Map, 3, 0.75, 0, 0.7, 1, 0); 
    	// R7
    	drawRect(Map, 3.5, 1.25, 0, 0.5, 1, 0); 
    	// R8
    	drawRect(Map, 3.25, 0.75, 0, 0.5, 0.5, 0); 

    	// IN RIGHT
    	drawRect(Map, (2.55+0.25), 0, 0, 1.5, 0.5, 0);     	
    	// IN LEFT
    	drawRect(Map, 5, -0.2, 0, 0.5, 2.5, 0);
}

int getCell(nav_msgs::OccupancyGrid Map, float x, float y)
{
// Environement
	float res = Map.info.resolution;
	int width = Map.info.width;
	float xO = Map.info.origin.position.x;
	float yO = Map.info.origin.position.y;
	int hCell = 0;
	int wCell = 0;
	int cell = 0;

// Algo
	hCell = (y-yO)/res;
	wCell = (x-xO)/res;
	cell = hCell*width + wCell;

	return cell;
}

int drawRect(nav_msgs::OccupancyGrid &Map, float x, float y, float theta, float height, float width, float margin)
{
// Environment
	static bool alreadyDone = false;
	if(alreadyDone && 0)
		return -1;
	alreadyDone = true;

	float res = Map.info.resolution;
	const int samplingMultiplier = 2;
	float totalDrawHeight = (2*margin+height);
	float totalDrawWidth = (2*margin+width);


	// ROS_INFO("totalDrawHeight %f", totalDrawHeight);
	// ROS_INFO("totalDrawWidth %f", totalDrawWidth);
	// ROS_INFO("__________________________________");

// Algo
	for (int i = 0; i < (int)(totalDrawWidth*samplingMultiplier/res); ++i)
	{
		for (int j = 0; j < (int)(totalDrawHeight*samplingMultiplier/res); ++j)
		{
			// ROS_INFO("%d:%d",i,j);
			// Define the gain for this point
			int gain = 0;
			// Outside of the rect (margin zones)
			if(	i < (int)(margin*samplingMultiplier/res)
				|| i > (int)((margin+width)*samplingMultiplier/res)
				|| j < (int)(margin*samplingMultiplier/res)
				|| j > (int)((margin+height)*samplingMultiplier/res))
			{
				// Variable gain, proportional to the distance with the rect
				// 	/!\ Temporarily to 50
				gain = 25;
			}
			// Into the rect
			else {
				// Static gain, set to maximum (100)
				gain = 100;
			}

			// ROS_INFO("Gain : %d",gain);

			// Get the coords
			float deltaX = -totalDrawWidth/2 + i*res/samplingMultiplier;
			float deltaY = totalDrawHeight/2 - j*res/samplingMultiplier;

			// ROS_INFO("deltaX : %f",deltaX);
			// ROS_INFO("deltaY : %f",deltaY);

			// ROS_INFO("dXc %f dYs %f",deltaX*cos(theta),deltaY*sin(theta));
			// ROS_INFO("dYc %f dXs %f",deltaY*cos(theta),deltaX*sin(theta));
			// float xP = x + deltaX*cos(theta) - deltaY*sin(theta);
			// float yP = y - deltaY*cos(theta) - deltaX*sin(theta);
			float xP = x + deltaX*cos(theta) + deltaY*sin(theta);
			float yP = y - deltaY*cos(theta) + deltaX*sin(theta);

			// ROS_INFO("xP : %f",xP);
			// ROS_INFO("yP : %f",yP);

			// Get the corresponding cell
			int cell = getCell(Map, xP, yP);

			// ROS_INFO("cell : %d",cell);

			// Fill it
			Map.data[cell] = gain;
		}

		// ROS_INFO("\t__________________________________");
	}

	return 0;
}

// TODO - ERASE zones
int eraseZone(nav_msgs::OccupancyGrid &Map, int zone)
{
	// Get bottom-left coord of zone
	float x=0;
	float y=0;
	// Right side
	if(zone>0 && zone<13) {
		x = ((zone-1)/4)*2;
		y = ((zone-1)%4)*1.5;
	}
	// Left side
	else if (zone<=24) {
		zone -=12;
		x = -((zone-1)/4)*2 - 2;
		y = ((zone-1)%4)*1.5;
	}
	else {
		return -1;
	}
	x+=0.001;
	y+=0.001;

	// Erase it
	for (int i = 0; i < (int)(2.0/Map.info.resolution); ++i)
	{
		for (int j = 0; j < (int)(1.5/Map.info.resolution); ++j)
		{
			float xP = x+i*Map.info.resolution;
			float yP = y+j*Map.info.resolution;

			// Get cell
			int cell = getCell(Map, xP, yP);

			// Erase it
			Map.data[cell] = 0;
		}
	}

	return 0;
}

// TODO - Get zones
int getZone(float x, float y)
{
	int zone = 0;

	// Right side
	if(x >= 0 && y >= 0) {
		// Anti-division par 0
		if(x==0) x=1;
		int w = (int)(x/2);

		// Anti-division par 0
		if(y==0) y=1;
		int h = (int)(y/1.5)+1;

		zone = w*4 + h;
	}
	// Left side
	else if (x < 0 && y >= 0) {
		int w = (int)(-x/2);

		// Anti-division par 0
		if(y==0) y=1;
		int h = (int)(y/1.5)+1;

		zone = w*4 + h + 12;
	}
	else {
		int zone = 0;
	}

	return zone;
}

// TODO - Dynamic
