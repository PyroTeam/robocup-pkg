/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>


/*========================================
=            Class Definition            =
========================================*/


bool processed = false;
Mat imgRef;


LectureFeu::LectureFeu()
:   it_(nh_),
  as_(nh_, ros::this_node::getName(), false),
  action_name_(ros::this_node::getName())
{
    // Ros topics
    image_sub_ = it_.subscribe("hardware/camera/platform_camera/image_raw", 1, &LectureFeu::imageCb, this);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/0_Flux_origine", "/image_raw");

    output_1_pub_ = it_.advertise("computerVision/lightSignalDetection/img_ouput_1", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/1_Ouput", "computerVision/lightSignalDetection/img_ouput_1");
    output_2_pub_ = it_.advertise("computerVision/lightSignalDetection/img_ouput_2", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/2_Output", "computerVision/lightSignalDetection/img_ouput_2");
    output_3_pub_ = it_.advertise("computerVision/lightSignalDetection/img_ouput_3", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/3_Output", "computerVision/lightSignalDetection/img_ouput_3");
    output_4_pub_ = it_.advertise("computerVision/lightSignalDetection/img_ouput_4", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/4_Output", "computerVision/lightSignalDetection/img_ouput_4");
    output_5_pub_ = it_.advertise("computerVision/lightSignalDetection/img_ouput_5", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/5_Output", "computerVision/lightSignalDetection/img_ouput_5");
    result_pub_ = it_.advertise("computerVision/lightSignalDetection/img_result", 1);
    nh_.setParam("computerVision/lightSignalDetection/image_result/list/6_Image_resultat", "computerVision/lightSignalDetection/img_result");

    // Images - init _origin, _origin_rbg, _result, etc
    initMembersImgs();

    // Ros action server
    as_.registerGoalCallback(boost::bind(&LectureFeu::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&LectureFeu::preemptCB, this));
    as_.start();
}

void LectureFeu::goalCB()
{
    nbImgProcessed_ = 0;
    red_last_results_=0;
    yellow_last_results_=0;
    green_last_results_=0;


    // Action Feedback
    beginOfProcessing_ = ros::Time::now();
    nbImgProcessed_ = 0;

    // accept the new goal
    as_.acceptNewGoal();

    feedback_.percent_complete = (ros::Time::now()-beginOfProcessing_).toSec() / minProcessTimeNeeded_;
    as_.publishFeedback(feedback_);
}

void LectureFeu::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();

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
    output_1_pub_.shutdown();
    output_2_pub_.shutdown();
    output_3_pub_.shutdown();
    output_4_pub_.shutdown();
    output_5_pub_.shutdown();
}

bool LectureFeu::ok()
{
    cv::waitKey(1);
    return nh_.ok();
}

/**
 * @brief CallBack fonction
 * @details [long description]
 *
 * @param msg [description]
 */
void LectureFeu::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // Quit if inactive action server
    if (!as_.isActive())
          return;

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

    // Conversions couleurs
    cv::cvtColor(_origin,_origin_rgb,CV_BGR2RGB);

    // Do the processing
    float timeElapsed = (ros::Time::now()-beginOfProcessing_).toSec();
    ++nbImgProcessed_;
    lectureFeu();

    // Save results
    red_last_results_+=((red_)?1:0);
    yellow_last_results_+=((yellow_)?1:0);
    green_last_results_+=((green_)?1:0);

    if(timeElapsed >= minProcessTimeNeeded_) {
        // Action feedback
        feedback_.percent_complete = 100;
        feedback_.images_processed = nbImgProcessed_;
        as_.publishFeedback(feedback_);

        // Good result only if we have enough images processed
        if (nbImgProcessed_/timeElapsed >= minNbImgProcessedPerSecond_) {
            // Action result
            comm_msg::LightSpec light;

            light.color = light.RED;
            if(red_last_results_ > nbImgProcessed_*0.66)
                light.state = light.ON;
            else if(red_last_results_ < nbImgProcessed_*0.3)
                light.state = light.OFF;
            else
                light.state = light.BLINK;
            result_.light_signal.push_back(light);

            light.color = light.YELLOW;
            if(yellow_last_results_ > nbImgProcessed_*0.66)
                light.state = light.ON;
            else if(yellow_last_results_ < nbImgProcessed_*0.3)
                light.state = light.OFF;
            else
                light.state = light.BLINK;
            result_.light_signal.push_back(light);

            light.color = light.GREEN;
            if(green_last_results_ > nbImgProcessed_*0.66)
                light.state = light.ON;
            else if(green_last_results_ < nbImgProcessed_*0.3)
                light.state = light.OFF;
            else
                light.state = light.BLINK;
            result_.light_signal.push_back(light);

            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        else {
            // Action result - all off because of abortion
            comm_msg::LightSpec light;

            light.color = light.RED;
            light.state = light.OFF;
            result_.light_signal.push_back(light);

            light.color = light.YELLOW;
            light.state = light.OFF;
            result_.light_signal.push_back(light);

            light.color = light.GREEN;
            light.state = light.OFF;
            result_.light_signal.push_back(light);

            //set the action state to aborted
            as_.setAborted(result_);
        }
    }
    else {
    // Action Feedback
        feedback_.percent_complete = (timeElapsed*100)/minProcessTimeNeeded_;
        // feedback_.percent_complete = timeElapsed;
        feedback_.images_processed = nbImgProcessed_;
        as_.publishFeedback(feedback_);
    }
}

void LectureFeu::initMembersImgs()
{
    // Images
    _origin.create(240, 320, CV_8UC3);
    cv::randu(_origin, cv::Scalar(0), cv::Scalar(256));
    _origin_treated.create(240, 320, CV_8UC3);
    cv::randu(_origin_treated, cv::Scalar(0), cv::Scalar(256));
    _origin_rgb.create(240, 320, CV_8UC3);
    cv::randu(_origin_rgb, cv::Scalar(0), cv::Scalar(256));
    _hsv.create(240, 320, CV_8UC3);
    cv::randu(_hsv, cv::Scalar(0), cv::Scalar(256));
    _thesholded.create(240, 320, CV_8UC3);
    cv::randu(_thesholded, cv::Scalar(0), cv::Scalar(256));

    _origin.create(240, 320, CV_8UC3);
    cv::randu(_origin, cv::Scalar(0), cv::Scalar(256));
    _output_1.create(240, 320, CV_8UC3);
    cv::randu(_output_1, cv::Scalar(0), cv::Scalar(256));
    _output_2.create(240, 320, CV_8UC3);
    cv::randu(_output_2, cv::Scalar(0), cv::Scalar(256));
    _output_3.create(240, 320, CV_8UC3);
    cv::randu(_output_3, cv::Scalar(0), cv::Scalar(256));
    _output_4.create(240, 320, CV_8UC3);
    cv::randu(_output_4, cv::Scalar(0), cv::Scalar(256));
    _output_5.create(240, 320, CV_8UC3);
    cv::randu(_output_5, cv::Scalar(0), cv::Scalar(256));
    _result.create(240, 320, CV_8UC3);
    cv::randu(_result, cv::Scalar(0), cv::Scalar(256));
}

/*==========  Fonctions assurants la detection et la lecture du feu  ==========*/
void LectureFeu::preTraitement(cv::Mat &imgToProcess)
{
// Filtrage
    cv::blur(imgToProcess, imgToProcess, cv::Size(5,5));
}

void LectureFeu::traitement(cv::Mat &imgToProcess)
{
	// TODO: DO ROI TREATMENT
}

void LectureFeu::publishResults(cv::Mat &imgToProcess)
{
// Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, imgToProcess).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, _output_1).toImageMsg();
    output_1_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, _output_2).toImageMsg();
    output_2_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, _output_3).toImageMsg();
    output_3_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, _output_4).toImageMsg();
    output_4_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, _output_5).toImageMsg();
    output_5_pub_.publish(msg);
}

void LectureFeu::lectureFeu(cv::Mat &imgToProcess)  // NB : il faudrait utiliser les parametres pour l'image à analyser
{
    red_=false;
    yellow_=false;
    green_=false;

    preTraitement(imgToProcess);
    imgToProcess.copyTo(_output_1);
    traitement(imgToProcess);
    imgToProcess.copyTo(_output_5);
    publishResults(imgToProcess);
}


/*==========  Detections Algorithms   ==========*/





/*==========  Utils  ==========*/

cv::Mat LectureFeu::singleToMultChannels(cv::Mat binary, int numChannels)
{
    // Add copies of binary img in a tab
    cv::Mat binTab[numChannels];
    for(int i=0;i<numChannels;++i) { binTab[i]=binary; }

    // Merge them in one img
    cv::Mat imgMultChannels;
    cv::merge(binTab, numChannels, imgMultChannels);

    return imgMultChannels;
}

cv::Mat LectureFeu::binaryThreshold(cv::Mat imgBgr, char channel)
{
// Environement
    std::string paramBaseName;
    std::stringstream paramStream;
    std::string acceptedChannels("BGRHSV");
    cv::Mat binary = cv::Mat(imgBgr.rows, imgBgr.cols, CV_8UC(1), Scalar::all(255));

    // Prepare params recuperation
    if(acceptedChannels.find(channel) == std::string::npos)
    {
        ROS_WARN_STREAM("LectureFeu::binaryThreshold : Unknown channel to threshold (Ch "<<channel<<")");
        return binary;
    }
    else
    {
        paramStream << "/trait_im/feu_tricolore/tmp/"<<channel<<"/threshold/";
        paramBaseName = paramStream.str();
    }

    // Recup params
    bool enabled;
    int min, max;
    nh_.param<bool>(paramBaseName+"enabled", enabled, true);
    nh_.param<int>(paramBaseName+"min", min, 0);
    nh_.param<int>(paramBaseName+"max", max, 255);

    ROS_INFO_STREAM(enabled << " " << min << " " << max);

// Algo

    // In case of HSV, convert colors
    std::string HSV("HSV");
    cv::Mat imgToProcess;
    if(HSV.find(channel) != std::string::npos) { cv::cvtColor(imgBgr, imgToProcess, CV_BGR2HSV); }
    else { imgBgr.copyTo(imgToProcess); }

    // Configure threshold
    int minCh1 = 0,
        minCh2 = 0,
        minCh3 = 0;
    int maxCh1 = 255,
        maxCh2 = 255,
        maxCh3 = 255;
    if(channel == 'H' || channel == 'B') { minCh1=min; maxCh1=max; }
    if(channel == 'S' || channel == 'G') { minCh2=min; maxCh2=max; }
    if(channel == 'V' || channel == 'R') { minCh3=min; maxCh3=max; }

    // Threshold
    if(enabled)
        cv::inRange(imgBgr,cv::Scalar(minCh1,minCh2,minCh3),cv::Scalar(maxCh1,maxCh2,maxCh3),binary);
    else
        cv::inRange(imgBgr,cv::Scalar(0,0,0),cv::Scalar(255,255,255),binary);

    return binary;
}

cv::Mat LectureFeu::calcHist(cv::Mat imgToHist, int channel, bool normalize)
{
    if(channel > imgToHist.channels())
    {
        ROS_WARN_STREAM("LectureFeu::calcHist : Requested channel "<<channel<<" on a "<<imgToHist.channels()<<" channel(s) image");
        return cv::Mat();
    }

    // Separate channels
    std::vector<cv::Mat> channels;
    cv::split(imgToHist, channels);

    // Number of bins
    int bins = 256;
    // Initalize histogram arrays
    cv::Mat hist = Mat::zeros(1, bins, CV_32SC1);

    // Calculate the histogram of the image
    for (int i = 0; i < imgToHist.rows; i++)
    {
        for (int j = 0; j < imgToHist.cols; j++)
        {
            uchar val;

            // Ignore black pixels
            if(imgToHist.at<Vec3b>(i,j)[0] == 0
                && imgToHist.at<Vec3b>(i,j)[1] == 0
                && imgToHist.at<Vec3b>(i,j)[2] == 0)
                continue;
            else
                val = imgToHist.at<Vec3b>(i,j)[channel];
            hist.at<float>(val) += 1;
        }
    }

    // Normalize
    if(normalize)
    {
        // Search max bin value
        int hmax;
        for (int j = 0; j < bins-1; j++)
            hmax = hist.at<float>(j) > hmax ? hist.at<float>(j) : hmax;

        for (int j = 0; j < bins-1; j++)
            hist.at<float>(j) /= hmax;
    }

    return hist;
}

cv::Mat LectureFeu::histToImg(cv::Mat hist)
{
    // Image for the histogram
    int histSize = 256;
    // int histSize = hist.cols;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Draw
    for( int i = 1; i < histSize; i++ )
    {
        // // Graduation
        // if(i%5 == 0)
        // line( histImage, Point( bin_w*(i), hist_h ) ,
        //                Point( bin_w*(i), 0 ),
        //                Scalar( 0, 30, 0), 2, 8, 0  );
        // if(i%20 == 0)
        // line( histImage, Point( bin_w*(i), hist_h ) ,
        //                Point( bin_w*(i), 0 ),
        //                Scalar( 30, 0, 0), 2, 8, 0  );

        // Espace couleur FEU
        // Rouge
        if(i<10 || (i>=170 && i<=180))
        line( histImage, Point( bin_w*(i), hist_h ) ,
                       Point( bin_w*(i), 0 ),
                       Scalar( 0, 0, 50), 2, 8, 0  );
        // Vert
        if(i<90 && i>=70)
        line( histImage, Point( bin_w*(i), hist_h ) ,
                       Point( bin_w*(i), 0 ),
                       Scalar( 0, 50, 0), 2, 8, 0  );
        // Jaune
        if(i<30 && i>=10)
        line( histImage, Point( bin_w*(i), hist_h ) ,
                       Point( bin_w*(i), 0 ),
                       Scalar( 0, 30, 50), 2, 8, 0  );

        // Histo
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)*hist_h) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)*hist_h) ),
                       Scalar( 200, 200, 200), 2, 8, 0  );
    }

    return histImage;
}



/*-----  End of Class Definition  ------*/