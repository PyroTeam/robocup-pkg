#ifndef _TRAIT_IM__LIGHT_DETECTION__H_
#define _TRAIT_IM__LIGHT_DETECTION__H_


#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>


/* Les deux bibliothèques nécessaires d'opencv :
    - cv.h contient les structures et fonctions de manipulation d'images
    - highgui.h contient les fonctions d'affichage des images
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>


#include <sys/time.h>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>
#include <sstream>


#include <trait_im_msg/processLightSignalAction.h>
#include <trait_im_msg/LightSignal.h>
#include <comm_msg/LightSpec.h>

#include "feu_tricolore/display-histogram.h"


class LightDetection
{
private:
    // ROS
    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    image_transport::Publisher m_result_pub;

    // OpenCV
    cv::Mat m_origin_treated, m_origin_rgb, m_hsv, m_thesholded;
    cv::Mat m_origin, m_output_1, m_output_2, m_output_3, m_output_4, m_output_5, m_resultimg;

    // Signals
    bool m_redTurnedOn, m_yellowTurnedOn, m_greenTurnedOn;
    int m_nbRedTurnedOn;
    int m_nbYellowTurnedOn;
    int m_nbGreenTurnedOn;

public:
    LightDetection();
    ~LightDetection();
    bool ok();
    void doLightDetection(){ doLightDetection(m_origin); };
    void doLightDetection(cv::Mat &imgToProcess);

private:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void initMembersImgs();

public:
    void preTraitement() { preTraitement(m_origin); }
    void preTraitement(cv::Mat &imgToProcess);
    void traitement() { traitement(m_origin_treated); }
    void traitement(cv::Mat &imgToProcess);
    void publishResultImages() { publishResultImages(m_resultimg); }
    void publishResultImages(cv::Mat &imgToProcess);

private:
    std::string m_action_name;

    actionlib::SimpleActionServer<trait_im_msg::processLightSignalAction> m_as;
    trait_im_msg::processLightSignalFeedback m_feedback;
    trait_im_msg::processLightSignalResult m_result;
    void goalCB();
    void preemptCB();

    ros::Time m_beginOfProcessing;
    int m_nbImgProcessed;

    // TODO: Add parameters
    static constexpr int m_minNbImgProcessedPerSecond = 10;
    static constexpr float m_minProcessTimeNeeded = 0.5;
};


#endif // _TRAIT_IM__LIGHT_DETECTION__H_
