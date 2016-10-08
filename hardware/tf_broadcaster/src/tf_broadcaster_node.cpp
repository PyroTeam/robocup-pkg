#include "tf_broadcaster/tf_broadcaster.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    ros::Rate r(50);

    // Subscribe to odom info
    ros::Subscriber sub_odom = nh.subscribe("hardware/odom", 1, &poseCallback);

    while(nh.ok()){

        ros::spinOnce();
        r.sleep();
    }
}
