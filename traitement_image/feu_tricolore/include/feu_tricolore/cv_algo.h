#ifndef _PYRO_TRAIT_IM__CV_ALGO__HEADER_
#define _PYRO_TRAIT_IM__CV_ALGO__HEADER_

/*==========  Includes  ==========*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <sstream>

#include "trait_im_msg/LightSpec.h"
#include "trait_im_msg/LightSignal.h"
#include "std_msgs/String.h"

#include "cv_utils.h"

/*==========  Namespaces  ==========*/
namespace enc = sensor_msgs::image_encodings;

/*=========================================
=            Class Declaration            =
=========================================*/

class LectureFeu
{
private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
  
    // OpenCV    
    cv::Mat _origin, _origin_rgb, _hsv, _result;

public:
    LectureFeu();
    ~LectureFeu();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void traitement();
    bool ok();
};

/*-----  End of Class Declaration  ------*/

#endif // _PYRO_TRAIT_IM__CV_ALGO__HEADER_