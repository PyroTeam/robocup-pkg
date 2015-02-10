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
    nh_.setParam("/feu_tricolore/image_result/list/0_Flux_origine", "/image_raw");
    result_pub_ = it_.advertise("/feu_tricolore/img_result", 1);
    nh_.setParam("/feu_tricolore/image_result/list/3_Image_resultat", "/feu_tricolore/img_result");
    hsv_pub_ = it_.advertise("/feu_tricolore/img_hsv", 1);
    nh_.setParam("/feu_tricolore/image_result/list/1_Image_hsv", "/feu_tricolore/img_hsv");
    before_morphops_pub_ = it_.advertise("/feu_tricolore/img_before_morphops", 1);
    nh_.setParam("/feu_tricolore/image_result/list/2_Image_seuillee", "/feu_tricolore/img_before_morphops");

    // Images
    _origin.create(240, 320, CV_8UC3);
    cv::randu(_origin, cv::Scalar(0), cv::Scalar(256));
    _origin_rgb.create(240, 320, CV_8UC3);
    cv::randu(_origin_rgb, cv::Scalar(0), cv::Scalar(256));
    _hsv.create(240, 320, CV_8UC3);
    cv::randu(_hsv, cv::Scalar(0), cv::Scalar(256));
    _thesholded.create(240, 320, CV_8UC3);
    cv::randu(_thesholded, cv::Scalar(0), cv::Scalar(256));
    _result.create(240, 320, CV_8UC3);
    cv::randu(_result, cv::Scalar(0), cv::Scalar(256));
}

LectureFeu::~LectureFeu()
{
    image_sub_.shutdown();
    result_pub_.shutdown();
    hsv_pub_.shutdown();
    before_morphops_pub_.shutdown();
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
// Conversions couleurs
    cv::cvtColor(_origin,_hsv,CV_BGR2HSV);
    cv::cvtColor(_origin,_origin_rgb,CV_BGR2RGB);

// Récupération des params
    int HSV_max_value;
    int HSV_min_value;
    bool enableHSV;

    int openingSize;
    int openingIterations;
    bool enableOpening;

    int closingSize;
    int closingIterations;
    bool enableClosing;
    if(nh_.hasParam("/feu_tricolore"))
    {
        nh_.getParam("/feu_tricolore/HSV_threshold/value/max", HSV_max_value);
        nh_.getParam("/feu_tricolore/HSV_threshold/value/min", HSV_min_value);
        nh_.getParam("/feu_tricolore/HSV_threshold/enabled", enableHSV);

        nh_.getParam("/feu_tricolore/opening/size", openingSize);
        nh_.getParam("/feu_tricolore/opening/iteration", openingIterations);
        nh_.getParam("/feu_tricolore/opening/enabled", enableOpening);

        nh_.getParam("/feu_tricolore/closing/size", closingSize);
        nh_.getParam("/feu_tricolore/closing/iteration", closingIterations);
        nh_.getParam("/feu_tricolore/closing/enabled", enableClosing);
    }
    else
    {
        HSV_max_value = 255;
        HSV_min_value = 230;
        enableHSV = true;
        nh_.setParam("/feu_tricolore/HSV_threshold/value/max", HSV_max_value);
        nh_.setParam("/feu_tricolore/HSV_threshold/value/min", HSV_min_value);
        nh_.setParam("/feu_tricolore/HSV_threshold/enabled", enableHSV);

        openingSize = 3;
        openingIterations = 3;
        enableOpening = true;
        nh_.setParam("/feu_tricolore/opening/size", openingSize);
        nh_.setParam("/feu_tricolore/opening/iteration", openingIterations);
        nh_.setParam("/feu_tricolore/opening/enabled", enableOpening);

        closingSize = 3;
        closingIterations = 3;
        enableClosing = true;
        nh_.setParam("/feu_tricolore/closing/size", closingSize);
        nh_.setParam("/feu_tricolore/closing/iteration", closingIterations);
        nh_.setParam("/feu_tricolore/closing/enabled", enableClosing);
    }
    openingSize=((openingSize<1)?1:openingSize*2+1);
    closingSize=((closingSize<1)?1:closingSize*2+1);

// Seuillage HSV
    cv::Mat hsv_threshold;
    if(enableHSV)
        cv::inRange(_hsv,cv::Scalar(0,0,HSV_min_value),cv::Scalar(255,255,HSV_max_value),hsv_threshold);
    else
        cv::inRange(_hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,255),hsv_threshold);

    cv::Mat _hsv_tab[] = {hsv_threshold, hsv_threshold, hsv_threshold};
    cv::Mat result_threshold;
    cv::merge(_hsv_tab, 3, result_threshold);
    cv::bitwise_and(_origin_rgb, result_threshold, _thesholded);

// Opération de fermeture
    cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); // ancien 3.3
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); //ancien 3.3 puis 8.8

    if(enableClosing)
    {
        for(int k=0;k<closingIterations;k++){
            erode(result_threshold,result_threshold,erodeElement);
            dilate(result_threshold,result_threshold,dilateElement);
        }        
    }

// Opération d'ouverture
    erodeElement = getStructuringElement(cv::MORPH_RECT,cv::Size(openingSize,openingSize)); // ancien 3.3
    //dilate with larger element so make sure object is nicely visible
    dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(openingSize+2,openingSize+2)); //ancien 3.3 puis 8.8

    if(enableOpening)
    {
        for(int k=0;k<openingIterations;k++){
            dilate(result_threshold,result_threshold,dilateElement);
            erode(result_threshold,result_threshold,erodeElement);
        }        
    }

    cv::bitwise_and(_origin_rgb, result_threshold, _result);
// Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _result).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _hsv).toImageMsg();
    hsv_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _thesholded).toImageMsg();
    before_morphops_pub_.publish(msg);
}

bool LectureFeu::ok()
{
    cv::waitKey(1);
    return nh_.ok();
}

/*-----  End of Class Definition  ------*/