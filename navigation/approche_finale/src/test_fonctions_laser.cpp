#include <ros/ros.h>
#include <unistd.h>
#include "laserScan.h"
#include "Point.h"
#include "fa_utils.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc,argv,"test_fonctions_laser");
	ros::NodeHandle n;
  ros::Publisher set_angle = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Time::init();
	ros::Rate loop_rate(1000);
	laserScan ls;
	loop_rate.sleep();
	geometry_msgs::Twist msg;
	std::vector<float> position_y(100);
	std::vector<float> pente(100);
	int max_points=0;
	int nearby=0;
	float d=0;
	int j=0;
	float pos=0;
	cout << "En attente d'un scan laser complet" << endl;
	while(ros::ok()){
		
    if(ls.getRanges().size() == 513){
			max_points = ls.max_number_points();
			nearby = ls.nearest_object();
			d = ls.distance_objet(max_points);
			pos = ls.position_y(nearby,d);
			Segment s = ls.build_segment(nearby);
			if(j < position_y.size()){
				if(pos > 0){
					position_y[j] = ls.position_y(nearby,d);
					j++;
					}
			}
			else{				
				if(pos>0){
					s = ls.build_segment(max_points);
					for(int i=0;i<position_y.size()-1;i++){
						position_y[i] = position_y[i+1];
						pente[i] = pente[i+1];
					}
					position_y[position_y.size()-1] = pos;
					pente[pente.size()-1] = s.get_pente();
					
				}
			}
			cout << "le laser se situe a " << moy(position_y) <<" m du bord gauche de l'objet le plus proche" << endl;
			cout << "pente = " << moy(pente) << endl;
			asservissement_angle(n,moy(pente));
			
			}
		}
		
	ros::spinOnce();
  loop_rate.sleep();
} 

