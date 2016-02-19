#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazsim_msgs/LightSignalDetection.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <trait_im_msg/LightSignal.h>
#include <deplacement_msg/Landmarks.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <time.h>

double g_x, g_y, g_z;
/* TODO: Find better solution */
bool g_init = false;
ros::Publisher g_pubOdom;
ros::Publisher g_pubLightSignal;
ros::Publisher g_pubMachines;
deplacement_msg::Landmarks g_landmarks;


#include <boost/bind.hpp>
typedef const boost::shared_ptr<gazsim_msgs::LightSignalDetection const> ConstLightSignalDetectionPtr;
typedef const boost::shared_ptr< ::gazebo_msgs::ModelStates const> ModelStatesConstPtr;
void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
void gpsCallback(ConstPosePtr &msg);
void lightSignalCallback(ConstLightSignalDetectionPtr &msg);
void machineInfoCallback(ModelStatesConstPtr &msg);

#define ROBOTINO_NAME "robotino_pyro"

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
	printf("Gazebo to ros Started\n");
	// Load gazebo
	gazebo::setupClient(argc, argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;
	std::string robotName;
	nh.param<std::string>("simuRobotNamespace", robotName, ROBOTINO_NAME);

	// Publish to a Gazebo topic
	gazebo::transport::PublisherPtr pub =
		// node->Advertise<gazebo::msgs::Vector3d>("/gazebo/pyro_2015/robotino_pyro/RobotinoSim/MotorMove/");
		node->Advertise<gazebo::msgs::Vector3d>("/gazebo/pyro_2015/" +robotName+ "/RobotinoSim/MotorMove/");

	// Wait for a subscriber to connect
	pub->WaitForConnection();

	// Subscriber
	ros::Subscriber subCmdVel = nh.subscribe("hardware/cmd_vel", 1, &cmdVelCallback);
	ros::Subscriber subModelStates = nh.subscribe("/gazebo/model_states", 1, &machineInfoCallback);
	g_pubOdom = nh.advertise<nav_msgs::Odometry>("hardware/odom", 1000);
	gazebo::transport::SubscriberPtr subGps = node->Subscribe("/gazebo/pyro_2015/" +robotName+ "/gazsim/gps/", &gpsCallback);
	gazebo::transport::SubscriberPtr subLightSignal = node->Subscribe("/gazebo/pyro_2015/" +robotName+ "/gazsim/light-signal/", &lightSignalCallback);
	// Publisher
	g_pubLightSignal = nh.advertise<trait_im_msg::LightSignal>("hardware/closest_light_signal", 1000);
	g_pubMachines = nh.advertise<deplacement_msg::Landmarks>("objectDetection/landmarks", 1000, true);
	g_init = true;

	// Publisher loop...replace with your own code.
	g_x=0; g_y=0; g_z=0;
	gazebo::math::Vector3 vect(g_x,g_y,g_z);
	while (ros::ok())
	{
		// Throttle Publication
		gazebo::common::Time::MSleep(100);

		// Generate a pose
		vect.Set(g_x, g_y, g_z);

		// Convert to a pose message
		gazebo::msgs::Vector3d msg;
		gazebo::msgs::Set(&msg, vect);

		pub->Publish(msg);
		ros::spinOnce();
	}

	// Make sure to shut everything down.
	gazebo::shutdown();
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	g_x=msg->linear.x;
	g_y=msg->linear.y;
	g_z=msg->angular.z;
}

void gpsCallback(ConstPosePtr &msg)
{
    static double noiseX, noiseY, noiseTheta;
    // double nx=0, ny=0, ntheta=0;
    double nx = 0.05* (double(rand())/double(RAND_MAX) - .55);
    double ny = 0.05 * (double(rand())/double(RAND_MAX) - .55);
    double ntheta = 0.01 * (double(rand())/double(RAND_MAX) - .55);

    ros::NodeHandle nh;
    std::string tf_prefix;
    nh.param<std::string>("simuRobotNamespace", tf_prefix, "");;
    if (tf_prefix.size() != 0)
        tf_prefix += "/";

    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id=tf_prefix+"base_link";
    odom_msg.header.frame_id=tf_prefix+"odom";
    odom_msg.header.stamp=ros::Time::now();

    noiseX+=nx*g_x;
    noiseY+=ny*g_y;
    noiseTheta+=ntheta*g_z;
    ROS_INFO("noiseX : %f", float(noiseX));
    odom_msg.pose.pose.position.x=msg->position().x()+noiseX;
    odom_msg.pose.pose.position.y=msg->position().y()+noiseY;
    odom_msg.pose.pose.position.z=0;

    odom_msg.pose.pose.orientation.x=msg->orientation().x();
    odom_msg.pose.pose.orientation.y=msg->orientation().y();
    odom_msg.pose.pose.orientation.z=msg->orientation().z();
    odom_msg.pose.pose.orientation.w=msg->orientation().w();
    double yaw = tf::getYaw(odom_msg.pose.pose.orientation) + noiseTheta;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);


    odom_msg.twist.twist.linear.x=g_x+nx*g_x;
    odom_msg.twist.twist.linear.y=g_y+ny*g_y;
    odom_msg.twist.twist.linear.z=0;
    odom_msg.twist.twist.angular.x=0;
    odom_msg.twist.twist.angular.y=0;
    odom_msg.twist.twist.angular.z=g_z+ntheta*g_z;
    if (g_init)
    {
        g_pubOdom.publish(odom_msg);
    }
}


void lightSignalCallback(ConstLightSignalDetectionPtr &msg)
{
	trait_im_msg::LightSignal lightSignals_msg;
	comm_msg::LightSpec lightSpec_msg;

	for(int i = 0; i < msg->lights_size(); ++i)
	{
		lightSpec_msg.color = msg->lights(i).color();
		lightSpec_msg.state = msg->lights(i).state();
		lightSignals_msg.lights.push_back(lightSpec_msg);
	}

	if (g_init)
		g_pubLightSignal.publish(lightSignals_msg);
}

void machineInfoCallback(ModelStatesConstPtr &msg)
{
	ros::NodeHandle nh;
	bool useMachineInfo = false;
	nh.param<bool>("objectDetection/useSimLandmarks", useMachineInfo, false);

	printf("MachineInfo Callback\n");
	if (!useMachineInfo)
		return;

	g_landmarks.header.stamp = ros::Time::now();
	g_landmarks.header.frame_id = "map";

	// go through all machines
	g_landmarks.landmarks.clear();
	for(int i = 0; i < msg->name.size(); i++){

		if (msg->name[i][0] != 'C' && msg->name[i][0] != 'M')
		{
			continue;
		}

		geometry_msgs::Pose2D pose;
		pose.x = msg->pose[i].position.x;
		pose.y = msg->pose[i].position.y;
		pose.theta = tf::getYaw(msg->pose[i].orientation);

		g_landmarks.landmarks.push_back(pose);
	}

	if (g_init)
		g_pubMachines.publish(g_landmarks);

	return;
}
