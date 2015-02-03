/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>

/*==========  Global Variables  ==========*/
// DEFAULT VALUES
// Red
    int H_MIN_red = 173;
    int H_MAX_red = 179;
    int S_MIN_red = 114;
    int S_MAX_red = HSV_MAX;
    int V_MIN_red = HSV_MIN;
    int V_MAX_red = HSV_MAX;
    int V_SEUIL_red = 150;

// Green
    int H_MIN_green = 40;
    int H_MAX_green = 82;
    int S_MIN_green = 75;
    int S_MAX_green = 185;
    int V_MIN_green = HSV_MIN;
    int V_MAX_green = HSV_MAX;
    int V_SEUIL_green = 60;

// Orange
    int H_MIN_orange = 15;
    int H_MAX_orange = 29;
    int S_MIN_orange = 120;
    int S_MAX_orange = 243;
    int V_MIN_orange = HSV_MIN;
    int V_MAX_orange = HSV_MAX;
    int V_SEUIL_orange = 191;

/*========================================
=            Class Definition            =
========================================*/


LectureFeu::LectureFeu()
  : it_(nh_)
{
    // Ros topics
    publisher_ = nh_.advertise<trait_im_msg::LightSignal>("/lecture_feu/result", 1000);
    image_sub_ = it_.subscribe("/image_raw", 1, &LectureFeu::imageCb, this);

    // Feu
    _lightRed.state = _lightRed.computedState = 0;
    _lightRed.timeSinceLastChange = _lightRed.timeSincePrecChange = 0;
    _lightOrange.state = _lightOrange.computedState = 0;
    _lightOrange.timeSinceLastChange = _lightOrange.timeSincePrecChange = 0;
    _lightGreen.state = _lightGreen.computedState = 0;
    _lightGreen.timeSinceLastChange = _lightGreen.timeSincePrecChange = 0;

    // Windows
    createTrackbars();
    // // cv::namedWindow(WINDOW);
    cv::namedWindow("Origin");
    // // cv::namedWindow("BGR");
    // // cv::namedWindow("Result");
    cv::namedWindow("Red");
    cv::namedWindow("Orange");
    cv::namedWindow("Green");

    // cv::resizeWindow("Red",320,240*3-86);
    // cv::resizeWindow("Orange",320,240*3-86);
    // cv::resizeWindow("Green",320,240*3-86);

    // cv::moveWindow("Red",0,0);
    // cv::moveWindow("Orange",320,0);
    // cv::moveWindow("Green",320*2,0);
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
    std_msgs::String str_msg;
    std::stringstream ss;

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

    ss << "Result : "<< _lightGreen.computedState << _lightOrange.computedState << _lightRed.computedState;
    str_msg.data = ss.str();


    trait_im_msg::LightSignal signal;
    trait_im_msg::LightSpec light;
    light.color = light.RED;light.state = _lightRed.computedState;
    signal.lights.push_back(light);
    light.color = light.YELLOW;light.state = _lightOrange.computedState;
    signal.lights.push_back(light);
    light.color = light.GREEN;light.state = _lightGreen.computedState;
    signal.lights.push_back(light);
    publisher_.publish(signal);

}

void LectureFeu::traitement()
{
// Environment
    // Base
    _origin.copyTo(_bgr);
    _origin.copyTo(_hsv);

    // Convert
    cv::cvtColor(_origin,_hsv,CV_RGB2HSV);
    cv::cvtColor(_origin,_bgr,CV_RGB2BGR);

    // Utils
    _origin.copyTo(_red);
    _origin.copyTo(_green);
    _origin.copyTo(_orange);
    _origin.copyTo(_result);

    // Time
    // struct timeval {
    //    time_t      tv_sec;     /* seconds */
    //    suseconds_t tv_usec;    /* microseconds */
    // };
    struct timeval tv;

// Algo
    // Threshold
    cv::inRange(_hsv,cv::Scalar(H_MIN_red,S_MIN_red,V_MIN_red),cv::Scalar(H_MAX_red,S_MAX_red,V_MAX_red),_red); 
    morphOps(_red);
    cv::inRange(_hsv,cv::Scalar(H_MIN_green,S_MIN_green,V_MIN_green),cv::Scalar(H_MAX_green,S_MAX_green,V_MAX_green),_green);
    morphOps(_green);
    cv::inRange(_hsv,cv::Scalar(H_MIN_orange,S_MIN_orange,V_MIN_orange),cv::Scalar(H_MAX_orange,S_MAX_orange,V_MAX_orange),_orange);
    morphOps(_orange);

    // Threshold Result
    cv::Mat res_red, res_orange, res_green;
    cv::Mat in_red[] = {_red, _red, _red};
    cv::Mat in_orange[] = {_orange, _orange, _orange};
    cv::Mat in_green[] = {_green, _green, _green};
    cv::merge(in_red, 3, res_red);
    cv::merge(in_orange, 3, res_orange);
    cv::merge(in_green, 3, res_green);
    cv::bitwise_and(_bgr, res_red, res_red);
    cv::bitwise_and(_bgr, res_orange, res_orange);
    cv::bitwise_and(_bgr, res_green, res_green);

    // Luminance
    int mean_value_r, mean_value_o, mean_value_g;

    mean_value_r = cv::sum(res_red)[2]/cv::countNonZero(_red);
    if(cv::countNonZero(_red) < 3)
        mean_value_r = 0;
    cv::putText(res_red,"Mean : "+intToString(mean_value_r)
        +((mean_value_r>V_SEUIL_red)?"  !! ON !!":"  ...  ...")
        ,cv::Point(0,20),1,2,cv::Scalar(255,255,255),2);
    mean_value_o = cv::sum(res_orange)[2]/cv::countNonZero(_orange);
    if(cv::countNonZero(_orange) < 3)
        mean_value_o = 0;
    cv::putText(res_orange,"Mean : "+intToString(mean_value_o)
        +((mean_value_o>V_SEUIL_orange)?"  !! ON !!":"  ...  ...")
        ,cv::Point(0,20),1,2,cv::Scalar(255,255,255),2);
    mean_value_g = cv::sum(res_green)[2]/cv::countNonZero(_green);
    if(cv::countNonZero(_green) < 3)
        mean_value_g = 0;
    cv::putText(res_green,"Mean : "+intToString(mean_value_g)
        +((mean_value_g>V_SEUIL_green)?"  !! ON !!":"  ...  ...")
        ,cv::Point(0,20),1,2,cv::Scalar(255,255,255),2);

    // Process state
    gettimeofday(&tv,NULL);
    long long int actualTime_ms = tv.tv_sec*1000+tv.tv_usec/1000;
    #define PERIOD_MEDIAN   500
    #define TIME_TOLERANCE_MS  100
    #define RATIO_MEDIAN    2
    #define RATIO_TOLERANCE 0.1
    if(_lightRed.state != (mean_value_r>V_SEUIL_red) || (actualTime_ms - _lightRed.timeSinceLastChange > PERIOD_MEDIAN*2))
    {
    // Store new state
        _lightRed.state = (mean_value_r>V_SEUIL_red);
    // Compute state
        long long int period = actualTime_ms - _lightRed.timeSincePrecChange;
        float ratio = (float)period / (actualTime_ms - _lightRed.timeSinceLastChange);
        if( period > (PERIOD_MEDIAN - TIME_TOLERANCE_MS) 
            && period < (PERIOD_MEDIAN + TIME_TOLERANCE_MS)
            && ratio > (RATIO_MEDIAN - RATIO_TOLERANCE)
            && ratio < (RATIO_MEDIAN + RATIO_TOLERANCE))
        {
            _lightRed.computedState = 2;
        }
        else
        {
            _lightRed.computedState = _lightRed.state;
        }
    // Save time
        _lightRed.timeSincePrecChange = _lightRed.timeSinceLastChange;
        _lightRed.timeSinceLastChange = actualTime_ms;
    }
    if(_lightOrange.state != (mean_value_o>V_SEUIL_orange) || (actualTime_ms - _lightOrange.timeSinceLastChange > PERIOD_MEDIAN*2))
    {    
    // Store new state
        _lightOrange.state = (mean_value_o>V_SEUIL_orange);
    // Compute state
        long long int period = actualTime_ms - _lightOrange.timeSincePrecChange;
        float ratio = (float)period / (actualTime_ms - _lightOrange.timeSinceLastChange);
        if( period > (PERIOD_MEDIAN - TIME_TOLERANCE_MS) 
            && period < (PERIOD_MEDIAN + TIME_TOLERANCE_MS)
            && ratio > (RATIO_MEDIAN - RATIO_TOLERANCE)
            && ratio < (RATIO_MEDIAN + RATIO_TOLERANCE))
        {
            _lightOrange.computedState = 2;
        }
        else
        {
            _lightOrange.computedState = _lightOrange.state;
        }
    // Save time
        _lightOrange.timeSincePrecChange = _lightOrange.timeSinceLastChange;
        _lightOrange.timeSinceLastChange = actualTime_ms;
    }
    if(_lightGreen.state != (mean_value_g>V_SEUIL_green) || (actualTime_ms - _lightGreen.timeSinceLastChange > PERIOD_MEDIAN*2))
    {
    // Store new state
        _lightGreen.state = (mean_value_g>V_SEUIL_green);
    // Compute state
        long long int period = actualTime_ms - _lightGreen.timeSincePrecChange;
        float ratio = (float)period / (actualTime_ms - _lightGreen.timeSinceLastChange);
        if( period > (PERIOD_MEDIAN - TIME_TOLERANCE_MS) 
            && period < (PERIOD_MEDIAN + TIME_TOLERANCE_MS)
            && ratio > (RATIO_MEDIAN - RATIO_TOLERANCE)
            && ratio < (RATIO_MEDIAN + RATIO_TOLERANCE))
        {
            _lightGreen.computedState = 2;
        }
        else
        {
            _lightGreen.computedState = _lightGreen.state;
        }
    // Save time
        _lightGreen.timeSincePrecChange = _lightGreen.timeSinceLastChange;
        _lightGreen.timeSinceLastChange = actualTime_ms;
    }

    // Compute state

    // Show
    // // cv::imshow(WINDOW,cv_ptr->image);
    cv::imshow("Origin", _bgr);
    // // cv::imshow("BGR", _bgr);
    cv::imshow("Red", res_red);
    cv::imshow("Orange", res_orange);
    cv::imshow("Green", res_green);
    // // cv::imshow("Result", _result);
    cv::waitKey(3);


    
    // cv_ptr->image = _result;
    // publisher_.publish(cv_ptr->toImageMsg());
}

bool LectureFeu::ok()
{
    return nh_.ok();
}

/*-----  End of Class Definition  ------*/

void on_trackbar( int, void* ){
    //This function gets called whenever a
    // trackbar position is changed
}

void createTrackbars()
{
    //create window for trackbars
    cvNamedWindow(trackbarWindowName_r,0);
    cvNamedWindow(trackbarWindowName_o,0);
    cvNamedWindow(trackbarWindowName_g,0);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH), 
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)                                ---->    ---->     ---->      
    cv::createTrackbar("H_MIN_r", trackbarWindowName_r, &H_MIN_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("H_MAX_r", trackbarWindowName_r, &H_MAX_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MIN_r", trackbarWindowName_r, &S_MIN_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MAX_r", trackbarWindowName_r, &S_MAX_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MIN_r", trackbarWindowName_r, &V_MIN_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MAX_r", trackbarWindowName_r, &V_MAX_red, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_SEUIL_r", trackbarWindowName_r, &V_SEUIL_red, HSV_MAX, on_trackbar);

    cv::createTrackbar("H_MIN_g", trackbarWindowName_g, &H_MIN_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("H_MAX_g", trackbarWindowName_g, &H_MAX_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MIN_g", trackbarWindowName_g, &S_MIN_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MAX_g", trackbarWindowName_g, &S_MAX_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MIN_g", trackbarWindowName_g, &V_MIN_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MAX_g", trackbarWindowName_g, &V_MAX_green, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_SEUIL_g", trackbarWindowName_g, &V_SEUIL_green, HSV_MAX, on_trackbar);

    cv::createTrackbar("H_MIN_o", trackbarWindowName_o, &H_MIN_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("H_MAX_o", trackbarWindowName_o, &H_MAX_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MIN_o", trackbarWindowName_o, &S_MIN_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("S_MAX_o", trackbarWindowName_o, &S_MAX_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MIN_o", trackbarWindowName_o, &V_MIN_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_MAX_o", trackbarWindowName_o, &V_MAX_orange, HSV_MAX, on_trackbar);
    cv::createTrackbar("V_SEUIL_o", trackbarWindowName_o, &V_SEUIL_orange, HSV_MAX, on_trackbar);
}