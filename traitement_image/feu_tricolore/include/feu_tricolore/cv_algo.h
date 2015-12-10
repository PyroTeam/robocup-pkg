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

#include "comm_msg/LightSpec.h"
#include "trait_im_msg/LightSignal.h"
#include "std_msgs/String.h"

#include "cv_utils.h"

#include "opencv2/features2d/features2d.hpp"
// #include "opencv2/nonfree/features2d.hpp"

#include <trait_im_msg/processLightSignalAction.h>
#include <actionlib/server/simple_action_server.h>

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
    image_transport::Publisher result_pub_;
    image_transport::Publisher output_1_pub_;
    image_transport::Publisher output_2_pub_;
    image_transport::Publisher output_3_pub_;
    image_transport::Publisher output_4_pub_;
    image_transport::Publisher output_5_pub_;

    // OpenCV
    cv::Mat _origin_treated, _origin_rgb, _hsv, _thesholded;
    cv::Mat _origin, _output_1, _output_2, _output_3, _output_4, _output_5, _result;

    // Signals
    bool red_, yellow_, green_;
    int red_last_results_;
    int yellow_last_results_;
    int green_last_results_;

public:
    LectureFeu();
    ~LectureFeu();
    bool ok();
    void lectureFeu(){ lectureFeu(_origin); };
    void lectureFeu(cv::Mat &imgToProcess);

private:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void initMembersImgs();

public:
    void preTraitement() { preTraitement(_origin); }
    void preTraitement(cv::Mat &imgToProcess);
    void traitement() { traitement(_origin_treated); }
    void traitement(cv::Mat &imgToProcess);
    void publishResults() { publishResults(_result); }
    void publishResults(cv::Mat &imgToProcess);

private:
    void hsvProcessing(cv::Mat imgToProcess);
    void hsvProcessing_V2(cv::Mat imgToProcess);
    void templateProcessing();
    int featureProcessing();

private:
    void opening(cv::Mat &imgToProcess, cv::Size size = cv::Size(3,3), int shape = cv::MORPH_RECT);
    void closing(cv::Mat &imgToProcess, cv::Size size = cv::Size(3,3), int shape = cv::MORPH_RECT);

// Parametres du filtrage HSV
private:
    int getHSV_max_value();
    int getHSV_min_value();
    bool getEnableHSV();

    int getOpeningSize();
    int getOpeningIterations();
    bool getEnableOpening();

    int getClosingSize();
    int getClosingIterations();
    bool getEnableClosing();

private:
    cv::Mat singleToMultChannels(cv::Mat binary, int numChannels = 3);
    cv::Mat binaryThreshold(cv::Mat imgBgr, char channel);
    cv::Mat morphOps(cv::Mat imgBinary, char channel);
    cv::Mat binaryMask(cv::Mat img3Channels, cv::Mat binaryMask);

private:
    cv::Mat calcHist(cv::Mat imgToHist, int channel=0, bool normalize=true);
    cv::Mat histToImg(cv::Mat hist);

private:
    actionlib::SimpleActionServer<trait_im_msg::processLightSignalAction> as_;
    std::string action_name_;
    trait_im_msg::processLightSignalFeedback feedback_;
    trait_im_msg::processLightSignalResult result_;
    void goalCB();
    void preemptCB();
    ros::Time beginOfProcessing_;
    int nbImgProcessed_;
    static const int minNbImgProcessedPerSecond_ = 10;
    static const float minProcessTimeNeeded_ = 0.5;
    int freakProcessing(std::string detector_str = "GFTT", std::string extractor_str = "BRISK", std::string matcher_str = "BruteForce-Hamming");
    int cascadeProcessing();
};

/*-----  End of Class Declaration  ------*/

#endif // _PYRO_TRAIT_IM__CV_ALGO__HEADER_
