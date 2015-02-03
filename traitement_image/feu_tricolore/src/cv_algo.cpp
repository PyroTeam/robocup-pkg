/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>

/*========================================
=            Class Definition            =
========================================*/


LectureFeu::LectureFeu()
  : it_(nh_)
{
    // Ros topics
    image_sub_ = it_.subscribe("/image_raw", 1, &LectureFeu::imageCb, this);

    // Windows
    cv::namedWindow("Origin");
    cv::namedWindow("Result");

    // Images
    _origin.create(240, 320, CV_8UC3);
    cv::randu(_origin, cv::Scalar(0), cv::Scalar(256));
    _origin_rgb.create(240, 320, CV_8UC3);
    cv::randu(_origin_rgb, cv::Scalar(0), cv::Scalar(256));
    _hsv.create(240, 320, CV_8UC3);
    cv::randu(_hsv, cv::Scalar(0), cv::Scalar(256));
    _result.create(240, 320, CV_8UC3);
    cv::randu(_result, cv::Scalar(0), cv::Scalar(256));
}

LectureFeu::~LectureFeu()
{

}

/**
 * @brief CallBack fonction
 * @details [long description]
 * 
 * @param msg [description]
 */
void LectureFeu::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
// Environment

// Algo
    // Get image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(_origin);

    traitement();    
}

void LectureFeu::traitement()
{
    cv::cvtColor(_origin,_hsv,CV_BGR2HSV);
    cv::cvtColor(_origin,_origin_rgb,CV_BGR2RGB);

    int HSV_max_value;
    int HSV_min_value;
    if(nh_.hasParam("/feu_tricolore"))
    {
        nh_.getParam("/feu_tricolore/HSV_threshold/value/max", HSV_max_value);
        nh_.getParam("/feu_tricolore/HSV_threshold/value/min", HSV_min_value);
    }
    else
    {
        HSV_max_value = 255;
        HSV_min_value = 0;
        nh_.setParam("/feu_tricolore/HSV_threshold/value/max", HSV_max_value);
        nh_.setParam("/feu_tricolore/HSV_threshold/value/min", HSV_min_value);
    }
    printf("%d || %d\n", HSV_max_value, HSV_min_value);

    cv::Mat hsv_threshold;
    cv::inRange(_hsv,cv::Scalar(0,0,HSV_min_value),cv::Scalar(255,255,HSV_max_value),hsv_threshold);

    cv::Mat _hsv_tab[] = {hsv_threshold, hsv_threshold, hsv_threshold};
    cv::Mat result_threshold;
    cv::merge(_hsv_tab, 3, result_threshold);
    cv::bitwise_and(_origin_rgb, result_threshold, _result);

    cv::imshow("Origin", _origin);
    cv::imshow("Result", _result);
    cv::imshow("HSV", _hsv);
}

bool LectureFeu::ok()
{
    cv::waitKey(1);
    return nh_.ok();
}

/*-----  End of Class Definition  ------*/