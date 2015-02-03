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

/*==========  Constantes  ==========*/
static const char WINDOW[] = "Image window";
static const char trackbarWindowName_r[] = "Red";
static const char trackbarWindowName_o[] = "Orange";
static const char trackbarWindowName_g[] = "Green";

/*==========  Defines  ==========*/
#define HSV_MIN 0
#define HSV_MAX 255

/*==========  Types  ==========*/
typedef struct 
{
    int state;
    long long int timeSinceLastChange;
    long long int timeSincePrecChange;
    int computedState;
}Feu;

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
    ros::Publisher publisher_;
  
    // OpenCV    
    cv::Mat _origin, _hsv, _bgr, _red, _orange, _green, _result;

    // Traitement
    Feu _lightRed, _lightOrange, _lightGreen;

public:
    LectureFeu();
    ~LectureFeu();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void traitement();
    bool ok();
};

/*-----  End of Class Declaration  ------*/


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param t [description]
 * @param d [description]
 */
void on_trackbar( int, void* );

/**
 * @brief [brief description]
 * @details [long description]
 */
void createTrackbars();

#endif // _PYRO_TRAIT_IM__CV_ALGO__HEADER_