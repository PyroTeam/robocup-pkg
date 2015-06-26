/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>


/**

    TODO:
    - Actionlib
    - Thread / actions concurrentes
    - Publish only when requested

**/


/*========================================
=            Class Definition            =
========================================*/



void detectAndDisplay( Mat frame );

bool processed = false;
Mat imgRef;


std::string cascade_dir("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/src/");
String cascade_name = cascade_dir + "cascade.xml";
CascadeClassifier cascade;

LectureFeu::LectureFeu()
  : it_(nh_),
  as_(nh_, ros::this_node::getName(), false),
  action_name_(ros::this_node::getName())
{
    // Ros topics
    image_sub_ = it_.subscribe("image_raw", 1, &LectureFeu::imageCb, this);
    nh_.setParam("feu_tricolore/image_result/list/0_Flux_origine", "/image_raw");

    output_1_pub_ = it_.advertise("feu_tricolore/img_ouput_1", 1);
    nh_.setParam("feu_tricolore/image_result/list/1_Ouput", "/feu_tricolore/img_ouput_1");
    output_2_pub_ = it_.advertise("feu_tricolore/img_ouput_2", 1);
    nh_.setParam("feu_tricolore/image_result/list/2_Output", "/feu_tricolore/img_ouput_2");
    output_3_pub_ = it_.advertise("feu_tricolore/img_ouput_3", 1);
    nh_.setParam("feu_tricolore/image_result/list/3_Output", "/feu_tricolore/img_ouput_3");
    output_4_pub_ = it_.advertise("feu_tricolore/img_ouput_4", 1);
    nh_.setParam("feu_tricolore/image_result/list/4_Output", "/feu_tricolore/img_ouput_4");
    output_5_pub_ = it_.advertise("feu_tricolore/img_ouput_5", 1);
    nh_.setParam("feu_tricolore/image_result/list/5_Output", "/feu_tricolore/img_ouput_5");
    result_pub_ = it_.advertise("feu_tricolore/img_result", 1);
    nh_.setParam("feu_tricolore/image_result/list/6_Image_resultat", "/feu_tricolore/img_result");

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


    if( !cascade.load( cascade_name ) ){ printf("--(!)Error loading\n"); return; };
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
    // hsvProcessing(imgToProcess);
    hsvProcessing_V2(imgToProcess);
    // templateProcessing();
    // featureProcessing();
    // freakProcessing();
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
    imgToProcess.copyTo(_output_4);
    publishResults(imgToProcess);
}


/*==========  Detections Algorithms   ==========*/

void LectureFeu::hsvProcessing(cv::Mat imgToProcess)
{
// Recupération des params
//  (actualisation des parametres a chaque nouvelle image et non pas pendant le traitement)
    int HSV_max_value = getHSV_max_value();
    int HSV_min_value = getHSV_min_value();
    bool enableHSV = getEnableHSV();

    int openingSize = getOpeningSize();
    int openingIterations = getOpeningIterations();
    bool enableOpening = getEnableOpening();

    int closingSize = getClosingSize();
    int closingIterations = getClosingIterations();
    bool enableClosing = getEnableClosing();

// Conversion couleur
    cv::Mat hsv;
    cv::cvtColor(imgToProcess,hsv,CV_BGR2HSV);

// Seuillage HSV
    // cv::Mat threshold_binary;
    // if(enableHSV)
    //     cv::inRange(hsv,cv::Scalar(0,0,HSV_min_value),cv::Scalar(255,255,HSV_max_value),threshold_binary);
    // else
    //     cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,255),threshold_binary);
    // cv::Mat threshold_binary;
    // if(enableHSV)
    //     cv::inRange(hsv,cv::Scalar(0,HSV_min_value,0),cv::Scalar(255,HSV_max_value,255),threshold_binary);
    // else
    //     cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,255),threshold_binary);
    cv::Mat threshold_binary;
    if(enableHSV)
        cv::inRange(hsv,cv::Scalar(0,HSV_min_value,0),cv::Scalar(255,HSV_max_value,255),threshold_binary);
    else
        cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(255,255,255),threshold_binary);

// Sauvegarde dans Ouput 2
    cv::Mat bin_tab[] = {threshold_binary, threshold_binary, threshold_binary};
    cv::Mat thresh_3canals;
    cv::merge(bin_tab, 3, thresh_3canals);
    thresh_3canals.copyTo(_output_2);

// Nettoyage - Operations morphologiques
    // Opération d'ouverture - 1
    if(enableOpening) {
        for(int k=0;k<openingIterations;k++) {
            opening(threshold_binary, cv::Size(openingSize,openingSize));
        }
    }

    // Opération de fermeture -1
    if(enableClosing) {
        for(int k=0;k<closingIterations;k++) {
            closing(threshold_binary, cv::Size(closingSize,closingSize));
        }
    }

    // Opération d'ouverture - 2
    if(enableOpening) {
        for(int k=0;k<openingIterations;k++) {
            opening(threshold_binary, cv::Size(openingSize,openingSize));
        }
    }

// Conversion de l'image resultat, un masque binaire, en image 3canaux
    cv::Mat binary_tab[] = {threshold_binary, threshold_binary, threshold_binary};
    cv::Mat threshold_3canals;
    cv::merge(binary_tab, 3, threshold_3canals);

// Sauvegarde dans Ouput 3
    threshold_3canals.copyTo(_output_3);

// Fusion du masque de seuillage avec l'image d'origine
    cv::bitwise_and(imgToProcess, threshold_3canals, imgToProcess);

// Sauvegarde dans Ouput 4
    threshold_3canals.copyTo(_output_4);

// // Seuillage couleur
//     // Separation en trois canaux
//     std::vector<cv::Mat> channels;
//     cv::split(imgToProcess, channels);
//     cv::Mat color_thresh;
//     cv::threshold(channels[0], color_thresh, 0, 255, THRESH_BINARY | THRESH_OTSU);
//     // cv::threshold(imgToProcess, color_thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 0);
//     cv::Mat color_thresh_binary_tab[] = {color_thresh, color_thresh, color_thresh};
//     cv::Mat color_thresh_3canals;
//     cv::merge(color_thresh_binary_tab, 3, color_thresh_3canals);

// // // Fusion du masque de seuillage avec l'image d'origine
//     cv::bitwise_and(imgToProcess, color_thresh_3canals, imgToProcess);

// cv::SimpleBlobDetector::Params params;
// params.minDistBetweenBlobs = 0;
// params.filterByInertia = false;
// params.filterByConvexity = false;
// params.filterByColor = true;
// params.filterByCircularity = false;
// params.filterByArea = false;
// params.minArea = 0;
// params.maxArea = 0;
// // ... any other params you don't want default value

// // set up and create the detector using the parameters
// cv::SimpleBlobDetector blob_detector(params);

// // detect!
// vector<cv::KeyPoint> keypoints;
// blob_detector.detect(imgToProcess, keypoints);

// // extract the x y coordinates of the keypoints:

// for (int i=0; i<keypoints.size(); i++){
//     float X = keypoints[i].pt.x;
//     float Y = keypoints[i].pt.y;

// Draw
// drawKeypoints( imgToProcess, keypoints, imgToProcess, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

// Histogram
    // cv::Mat imgSeuillee;
    // cv::inRange(imgToProcess,cv::Scalar(0,0,HSV_min_value),cv::Scalar(255,255,HSV_max_value),imgSeuillee);
    // cv::Mat img_tab[] = {imgSeuillee, imgSeuillee, imgSeuillee};
    // cv::Mat img_3canals;
    // cv::merge(imgToProcess, 3, img_3canals);

    /// Separate the image in 3 places ( B, G and R )
    vector<Mat> hsv_channels;
    split(imgToProcess, hsv_channels);
    imgToProcess.copyTo(_output_3);

    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for H,S,V) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    Mat h_hist, s_hist, v_hist;

    // /// Compute the histograms:
    range[0] = 10; range[1] = 256;
    cv::calcHist( &hsv_channels[0], 1, 0, Mat(), v_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::calcHist( &hsv_channels[1], 1, 0, Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::calcHist( &hsv_channels[2], 1, 0, Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate );

    // Calc hist manually
    int nc = imgToProcess.channels();   // number of channels
    int bins = 256;                     // number of bins
    vector<Mat> hist(nc);
    // Initalize histogram arrays
    for (int i = 0; i < hist.size(); i++)
        hist[i] = Mat::zeros(1, bins, CV_32SC1);
    // Calculate the histogram of the image
    for (int i = 0; i < imgToProcess.rows; i++)
    {
        for (int j = 0; j < imgToProcess.cols; j++)
        {
            for (int k = 0; k < nc; k++)
            {
                uchar val;
                if(nc == 1)
                {
                    if(imgToProcess.at<uchar>(i,j) == 0)
                        continue;
                    else
                        val = imgToProcess.at<uchar>(i,j);
                }
                else
                {
                    if((imgToProcess.at<Vec3b>(i,j)[0] == 0
                        && imgToProcess.at<Vec3b>(i,j)[1] == 0
                        && imgToProcess.at<Vec3b>(i,j)[2] == 0)
                        || imgToProcess.at<Vec3b>(i,j)[1] > 300 //235
                        || imgToProcess.at<Vec3b>(i,j)[1] < 0
                        || imgToProcess.at<Vec3b>(i,j)[0] > 300)
                        continue;
                    else
                        val = imgToProcess.at<Vec3b>(i,j)[k];
                }
                hist[k].at<float>(val) += 1;
            }
        }
    }

    // For each histogram arrays, obtain the maximum (peak) value
    // Needed to normalize the display later
    int hmax[3] = {0,0,0};
    for (int i = 0; i < nc; i++)
    {
        for (int j = 0; j < bins-1; j++)
            hmax[i] = hist[i].at<float>(j) > hmax[i] ? hist[i].at<float>(j) : hmax[i];
    }
    // Draw the histograms for H, S and V
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(h_hist, h_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(s_hist, s_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(v_hist, v_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    // // Compte les classes avant le filtrage
    int green_pix= 0, red_pix = 0, yellow_pix;
    // for( int i = 1; i < histSize; i++ )
    // {
    //     if(i < 50 || i >= 220)
    //         red_pix += v_hist.at<float>(i);
    //     else if(i >=140)
    //         yellow_pix += v_hist.at<float>(i);
    //     else
    //         green_pix += v_hist.at<float>(i);
    // }
    // ROS_INFO_STREAM("Red : " << red_pix << " Yellow : " << yellow_pix << " Green : " << green_pix);

    // // Filtrage de l'histo Hue
    // #define INT_WIDTH 20
    // float integral = 0;
    // for( int i = 1; i < histSize; i++ )
    // {
    //     // Somme des 10 dernieres colonnes de l'histogram
    //     integral += v_hist.at<float>(i);
    //     if(i>=INT_WIDTH+1)
    //     {
    //         integral -= v_hist.at<float>(i-INT_WIDTH);
    //         v_hist.at<float>(i-(INT_WIDTH/2)) = integral/INT_WIDTH;
    //     }
    // }

    // Compte les classes apres le filtrage
    green_pix= 0; red_pix = 0; yellow_pix = 0;
    for( int i = 1; i < histSize; i++ )
    {
        if(i < 0 || i >= 235)
            red_pix += hist[2].at<float>(i);
        else if(i >=215)
            yellow_pix += hist[2].at<float>(i);
        else
            green_pix += hist[2].at<float>(i);
    }
    // ROS_INFO_STREAM("Red : " << red_pix << " Yellow : " << yellow_pix << " Green : " << green_pix);
    int sum = red_pix + yellow_pix + green_pix;
    if(sum!=0)
    {
        red_pix = red_pix*100/sum;
        yellow_pix = yellow_pix*100/sum;
        green_pix = green_pix*100/sum;
    }
    else
    {
        red_pix = yellow_pix = green_pix = 0;
    }
    // ROS_INFO_STREAM(" --- Red : " << red_pix << "% Yellow : " << yellow_pix << "% Green : " << green_pix << "%");

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[2].at<float>(i-1)*hist_h/hmax[2]) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist[2].at<float>(i)*hist_h/hmax[2]) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[1].at<float>(i-1)*hist_h/hmax[1]) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist[1].at<float>(i)*hist_h/hmax[1]) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[0].at<float>(i-1)*hist_h/hmax[0]) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist[0].at<float>(i)*hist_h/hmax[0]) ),
                       Scalar( 255, 255, 255), 2, 8, 0  );
    }

// Sauvegarde dans Ouput 5
    #define CHANNEL     1
    if(openingIterations<0 || openingIterations > 2) openingIterations =0;
    cv::Mat img_tab[] = {hsv_channels[openingIterations], hsv_channels[openingIterations], hsv_channels[openingIterations]};
    cv::Mat img_3canals;
    cv::merge(img_tab, 3, img_3canals);
    img_3canals.copyTo(_output_3);

    histImage.copyTo(_output_5);

    // showHistogram(imgToProcess);
}


void LectureFeu::hsvProcessing_V2(cv::Mat imgToProcess)
{
    // Recup img
    cv::Mat BGR = imgToProcess;
    cv::Mat HSV;
    cvtColor(imgToProcess, HSV, CV_BGR2HSV);

    // Split channels
    vector<Mat> hsvChannels;
    split(HSV, hsvChannels);
    vector<Mat> bgrChannels;
    split(BGR, bgrChannels);

    // Transform to 3 channels grayscales imgs
    cv::Mat imgB = singleToMultChannels(bgrChannels[0],3);
    cv::Mat imgG = singleToMultChannels(bgrChannels[1],3);
    cv::Mat imgR = singleToMultChannels(bgrChannels[2],3);

    cv::Mat imgH = singleToMultChannels(hsvChannels[0],3);
    cv::Mat imgS = singleToMultChannels(hsvChannels[1],3);
    cv::Mat imgV = singleToMultChannels(hsvChannels[2],3);

    // // Show results
    // imshow("Blue", imgB); moveWindow("Blue", 400*0, 300*0);
    // imshow("Green", imgG); moveWindow("Green", 400*1, 300*0);
    // imshow("Red", imgR); moveWindow("Red", 400*2, 300*0);

    // imshow("Hue", imgH); moveWindow("Hue", 400*0, 300*1);
    // imshow("Saturation", imgS); moveWindow("Saturation", 400*1, 300*1);
    // imshow("Value", imgV); moveWindow("Value", 400*2, 300*1);

    // Filter each channels
    cv::Mat binB = binaryThreshold(BGR,'B');
    cv::Mat binG = binaryThreshold(BGR,'G');
    cv::Mat binR = binaryThreshold(BGR,'R');

    cv::Mat binH = binaryThreshold(BGR,'H');
    cv::Mat binS = binaryThreshold(BGR,'S');
    cv::Mat binV = binaryThreshold(BGR,'V');

    // // Show results
    // imshow("Blue", binB); moveWindow("Blue", 400*0, 300*0);
    // imshow("Green", binG); moveWindow("Green", 400*1, 300*0);
    // imshow("Red", binR); moveWindow("Red", 400*2, 300*0);

    // imshow("Hue", binH); moveWindow("Hue", 400*0, 300*1);
    // imshow("Saturation", binS); moveWindow("Saturation", 400*1, 300*1);
    // imshow("Value", binV); moveWindow("Value", 400*2, 300*1);

    // Morphops on it
    binB = morphOps(binB,'B');
    binG = morphOps(binG,'G');
    binR = morphOps(binR,'R');

    binH = morphOps(binH,'H');
    binS = morphOps(binS,'S');
    binV = morphOps(binV,'V');

    // // Show results
    // imshow("Blue", binB); moveWindow("Blue", 400*0, 300*0);
    // imshow("Green", binG); moveWindow("Green", 400*1, 300*0);
    // imshow("Red", binR); moveWindow("Red", 400*2, 300*0);

    // imshow("Hue", binH); moveWindow("Hue", 400*0, 300*1);
    // imshow("Saturation", binS); moveWindow("Saturation", 400*1, 300*1);
    // imshow("Value", binV); moveWindow("Value", 400*2, 300*1);

    // Merge with original
    imgB = binaryMask(imgB, binB);
    imgG = binaryMask(imgG, binG);
    imgR = binaryMask(imgR, binR);

    imgH = binaryMask(imgH, binH);
    imgS = binaryMask(imgS, binS);
    imgV = binaryMask(imgV, binV);

    // // Show results
    // imshow("Blue", imgB); moveWindow("Blue", 400*0, 300*0);
    // imshow("Green", imgG); moveWindow("Green", 400*1, 300*0);
    // imshow("Red", imgR); moveWindow("Red", 400*2, 300*0);

    // imshow("Hue", imgH); moveWindow("Hue", 400*0, 300*1);
    // imshow("Saturation", imgS); moveWindow("Saturation", 400*1, 300*1);
    // imshow("Value", imgV); moveWindow("Value", 400*2, 300*1);

    // Mask originals images
    cv::Mat hueMasked = binaryMask(imgH, binG);
    cv::Mat satMasked = binaryMask(imgS, binG);

    // Histograms
    cv::Mat hueHist = calcHist(hueMasked, 0, true);
    cv::Mat hueHistImg = histToImg(calcHist(hueMasked, 0, true));
    cv::Mat satHist = histToImg(calcHist(satMasked, 0, true));

    // // Show results
    // imshow("hueMasked", hueMasked); moveWindow("hueMasked", 400*0, 300*2);
    // imshow("satMasked", satMasked); moveWindow("satMasked", 400*1, 300*2);
    // imshow("hueHist", hueHistImg); moveWindow("hueHist", 400*2, 300*2);
    // imshow("satHist", satHist); moveWindow("satHist", 400*2+800, 300*2);

    // Count the red, yellow and green pixels
    float green_pix= 0, red_pix = 0, yellow_pix= 0;
    float green_pix_max= 0, red_pix_max = 0, yellow_pix_max= 0;
    float sum = 0;
    for( int i = 0; i < 180; i++ )
    {
        if(i >= 170 || i < 10 ) {
            red_pix += cvRound(hueHist.at<float>(i)*10);
            if (cvRound(hueHist.at<float>(i)*10 > red_pix_max))
                red_pix_max = cvRound(hueHist.at<float>(i)*10);
        }
        else if(i >=10 && i < 30) {
            yellow_pix += cvRound(hueHist.at<float>(i)*10);
            if (cvRound(hueHist.at<float>(i)*10 > yellow_pix_max))
                yellow_pix_max = cvRound(hueHist.at<float>(i)*10);
        }
        else if(i >=70 && i < 90) {
            green_pix += cvRound(hueHist.at<float>(i)*10);
            if (cvRound(hueHist.at<float>(i)*10 > green_pix_max))
                green_pix_max = cvRound(hueHist.at<float>(i)*10);
        }

        sum += cvRound(hueHist.at<float>(i)*10);
    }

    // ROS_INFO_STREAM("Red : " << red_pix << " Yellow : " << yellow_pix << " Green : " << green_pix);
    // ROS_INFO_STREAM("Red : " << red_pix/sum*100 << "% Yellow : " << yellow_pix/sum*100 << "% Green : " << green_pix/sum*100<<"% Sum "<<sum);
    // ROS_INFO_STREAM("Red : " << red_pix_max << " Yellow : " << yellow_pix_max << " Green : " << green_pix_max);

    // Extact a result
    if(red_pix > 10 && red_pix_max >= 6)
        red_=true;
    if(green_pix > 10 && green_pix_max >= 6)
        green_=true;
    if(yellow_pix > 10 && yellow_pix_max >= 6)
        yellow_=true;
    // ROS_INFO_STREAM("LIGHTS ON : "<<((red_)?"RED ":"")<<((yellow_)?"YELLOW ":"")<<((green_)?"GREEN ":""));
}

void LectureFeu::templateProcessing()
{
    // Open the template img
    std::string base_dir("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/img/joao_pessoa/");
    std::vector<cv::Mat> template_imgs;
    template_imgs.push_back(imread(base_dir + "TL_templ_000.jpg"));
    template_imgs.push_back(imread(base_dir + "TL_templ_001.jpg"));
    template_imgs.push_back(imread(base_dir + "TL_templ_010.jpg"));
    template_imgs.push_back(imread(base_dir + "TL_templ_110.jpg"));
    template_imgs.push_back(imread(base_dir + "TL_templ_111.jpg"));


    cv::Point true_matchLoc;
    double minMax = -1;
    cv::Mat templ;

    for(std::vector<cv::Mat>::iterator it = template_imgs.begin(); it != template_imgs.end(); it++)
    {
        templ = *it;

        /// "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
        int match_method = 0;
        nh_.param<int>("feu_tricolore/opening/size", match_method, 0);

        /// Create the result matrix
        int result_cols =  _origin.cols - templ.cols + 1;
        int result_rows = _origin.rows - templ.rows + 1;

        cv::Mat result;
        result.create( result_cols, result_rows, CV_32FC1 );

        /// Do the Matching and Normalize
        cv::matchTemplate( _origin, templ, result, match_method );
        // cv::normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

        /// Localizing the best match with minMaxLoc
        double minVal; double maxVal;
        cv::Point minLoc; cv::Point maxLoc;
        cv::Point matchLoc;

        cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ) {
            matchLoc = minLoc;
            if(minMax == -1 || minVal < minMax) {
                minMax = minVal;
                true_matchLoc = matchLoc;
            }
        }
        else {
            matchLoc = maxLoc;
            if(minMax == -1 || maxVal > minMax) {
                minMax = maxVal;
                true_matchLoc = matchLoc;
            }
        }
    }

    /// Show me what you got
    _origin_rgb.copyTo(_thesholded);
    _origin_rgb.copyTo(_result);
    rectangle( _thesholded, true_matchLoc, cv::Point( true_matchLoc.x + templ.cols , true_matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
    rectangle( _result, true_matchLoc, cv::Point( true_matchLoc.x + templ.cols , true_matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

    // Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _result).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _hsv).toImageMsg();
    output_1_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _thesholded).toImageMsg();
    output_2_pub_.publish(msg);
}

int LectureFeu::featureProcessing()
{
    std::string base_dir("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/img/joao_pessoa/");
    cv::Mat img_1 = imread( base_dir + "TL_templ_001.jpg", CV_LOAD_IMAGE_COLOR);
    // cv::Mat img_1 = imread( base_dir + "000.jpg", CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2;
    _origin.copyTo(img_2);

    if( !img_1.data || !img_2.data ) { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    // Ptr<SURF> detector = SURF::create( minHessian );
    Ptr<FeatureDetector> detector = FeatureDetector::create("MSER");

    // int tmp_1, tmp_2, tmp_3;
    // nh_.param<int>("feu_tricolore/closing/size", tmp_1, 400);
    // nh_.param<int>("feu_tricolore/opening/size", tmp_2, 4);
    // nh_.param<int>("feu_tricolore/opening/iteration", tmp_3, 2);

    // detector->hessianThreshold = 0;
    // detector->nOctaves = 4;
    // detector->nOctaveLayers = 2;
    // detector->extended = 1;     // extended descriptors
    // detector->upright = 0;      // 1 for not computing orientation

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    detector->detect( img_1, keypoints_1 );
    detector->detect( img_2, keypoints_2 );
    // detector->detectAndCompute( img_1, keypoints_1, descriptors_1);
    // detector->detectAndCompute( img_2, keypoints_2, descriptors_2);

    //-- Draw keypoints
    Mat img_keypoints_1; Mat img_keypoints_2;

    drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


    //-- Show detected (drawn) keypoints
    // imshow("Keypoints 1", img_keypoints_1 );
    // imshow("Keypoints 2", img_keypoints_2 );

    // Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, img_keypoints_2).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, img_keypoints_1).toImageMsg();
    output_1_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, img_1).toImageMsg();
    output_2_pub_.publish(msg);
}


int LectureFeu::freakProcessing(std::string detector_str, std::string extractor_str, std::string matcher_str)
{
    ROS_INFO_STREAM("\n__________\nFreakProcessing\n-------------");
    ROS_INFO_STREAM("Detector : " << detector_str);
    ROS_INFO_STREAM("Extractor: " << extractor_str);
    ROS_INFO_STREAM("Matcher : " << matcher_str << "\n***\n");

    Mat imgObject, imgScene;
    _origin.copyTo(imgScene);
    if(!imgRef.data)
    {
        ROS_INFO("NEW IMAGE");
        getRectSubPix(imgScene,Size(60,150),Point(150,150),imgObject);
        imgObject.copyTo(imgRef);
    }
    else
    {
        imgRef.copyTo(imgObject);
    }
    // std::string base_dir("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/img/joao_pessoa/");
    // imgObject = imread( base_dir + "TL_templ_001.jpg", CV_LOAD_IMAGE_COLOR);
    cvtColor(imgObject, imgObject, CV_BGR2GRAY);
    cvtColor(imgScene, imgScene, CV_BGR2GRAY);

    std::vector<KeyPoint> keypointsObject, keypointsScene;
    std::vector<KeyPoint> keypointsObjectSave, keypointsSceneSave;
    Mat descriptorsObject, descriptorsScene;
    std::vector<DMatch> matches;

    // DETECTION
    // Any openCV detector such as
    // SurfFeatureDetector detector(0,1);
    // SurfFeatureDetector detector(0,4);
    // MserFeatureDetector detector();
    // Ptr<FeatureDetector> detector = FeatureDetector::create("MSER");
    /**
     * "FAST" – FastFeatureDetector
     * "STAR" – StarFeatureDetector
     * "SIFT" – SIFT (nonfree module)
     * "SURF" – SURF (nonfree module)
     * "ORB" – ORB
     * "BRISK" – BRISK
     * "MSER" – MSER
     * "GFTT" – GoodFeaturesToTrackDetector
     * "HARRIS" – GoodFeaturesToTrackDetector with Harris detector enabled
     * "Dense" – DenseFeatureDetector
     * "SimpleBlob" – SimpleBlobDetector
     */
    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_str);

    // DESCRIPTOR
    // Our proposed FREAK descriptor
    // (roation invariance, scale invariance, pattern radius corresponding to SMALLEST_KP_SIZE,
    // number of octaves, optional vector containing the selected pairs)
    // FREAK extractor(true, true, 22, 4, std::vector<int>());
    // FREAK extractor;
    // MSER extractor;

    /**
     * SIFT" – SIFT (nonfree module)
     * SURF" – SURF (nonfree module)
     * BRIEF" – BriefDescriptorExtractor
     * BRISK" – BRISK
     * ORB" – ORB
     * FREAK" – FREAK
     */
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(extractor_str);

    // MATCHER
    // The standard Hamming distance can be used such as
    // BruteForceMatcher<Hamming> matcher;
    // or the proposed cascade of hamming distance using SSSE3
    // BFMatcher matcher(NORM_HAMMING);
    /**
     * BruteForce (it uses L2 )
     * BruteForce-L1
     * BruteForce-Hamming
     * BruteForce-Hamming(2)
     * FlannBased
     */
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(matcher_str);

    // detect
    // double t = (double)getTickCount();
    detector->detect( imgObject, keypointsObject );
    detector->detect( imgScene, keypointsScene );
    keypointsObjectSave = keypointsObject;
    keypointsSceneSave = keypointsScene;
    // t = ((double)getTickCount() - t)/getTickFrequency();
    // std::cout << "detection time [s]: " << t/1.0 << std::endl;
    ROS_INFO_STREAM("Found " << keypointsObject.size() << " keyPoints for Object and " << keypointsScene.size() << " for Scene");

    // extract
    // t = (double)getTickCount();
    extractor->compute( imgObject, keypointsObject, descriptorsObject );
    extractor->compute( imgScene, keypointsScene, descriptorsScene );
    // t = ((double)getTickCount() - t)/getTickFrequency();
    // std::cout << "extraction time [s]: " << t << std::endl;
    ROS_INFO_STREAM("Keeped " << keypointsObject.size() << " keyPoints for Object and " << keypointsScene.size() << " for Scene");

    // match
    // t = (double)getTickCount();
    matcher->match(descriptorsObject, descriptorsScene, matches);
    // t = ((double)getTickCount() - t)/getTickFrequency();
    // std::cout << "matching time [s]: " << t << std::endl;
    ROS_INFO_STREAM("Found " << matches.size() << " matches");

    // // Draw matches
    // Mat imgMatch;
    // drawMatches(imgObject, keypointsObject, imgScene, keypointsScene, matches, imgMatch);
    // Draw keypoints
    Mat imgKeypointA, imgKeypointB;
    drawKeypoints( imgObject, keypointsObjectSave, imgKeypointA, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( imgScene, keypointsSceneSave, imgKeypointB, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptorsObject.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    ROS_INFO("-- Max dist : %f ", max_dist );
    ROS_INFO("-- Min dist : %f \n\n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptorsObject.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }

    Mat imgMatches;
    drawMatches( imgObject, keypointsObject, imgScene, keypointsScene,
               good_matches, imgMatches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypointsObject[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypointsScene[ good_matches[i].trainIdx ].pt );
    }

    Mat H;
    if(obj.size() >= 4)
    {
        H = findHomography( obj, scene, CV_RANSAC );
    }
    else
    {
        ROS_WARN("findHomography : not enough keyPoints (only %ld)", obj.size());
        processed = true;
        return -1;
    }

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgObject.cols, 0 );
    obj_corners[2] = cvPoint( imgObject.cols, imgObject.rows ); obj_corners[3] = cvPoint( 0, imgObject.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);




    Mat imgDrawn;
    imgMatches.copyTo(imgDrawn);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( imgDrawn, scene_corners[0] + Point2f( imgObject.cols, 0), scene_corners[1] + Point2f( imgObject.cols, 0), Scalar::all(0), 4 );
    line( imgDrawn, scene_corners[1] + Point2f( imgObject.cols, 0), scene_corners[2] + Point2f( imgObject.cols, 0), Scalar::all(0), 4 );
    line( imgDrawn, scene_corners[2] + Point2f( imgObject.cols, 0), scene_corners[3] + Point2f( imgObject.cols, 0), Scalar::all(0), 4 );
    line( imgDrawn, scene_corners[3] + Point2f( imgObject.cols, 0), scene_corners[0] + Point2f( imgObject.cols, 0), Scalar::all(0), 4 );


    //-- Show detected (drawn) keypoints
    // imshow("Keypoints A", imgKeypointA );
    // imshow("Keypoints B", imgKeypointB );

    // Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, imgDrawn).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, imgKeypointA).toImageMsg();
    output_1_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, imgKeypointB).toImageMsg();
    output_2_pub_.publish(msg);

    processed = true;

    return 0;
}

int LectureFeu::cascadeProcessing()
{
    detectAndDisplay(_origin);

    processed = true;

    return 0;
}


/*==========  Utils  ==========*/

void LectureFeu::opening(cv::Mat &imgToProcess, cv::Size size, int shape)
{
    cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT,size);
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(size.width+2, size.height+2));

    dilate(imgToProcess,imgToProcess,dilateElement);
    erode(imgToProcess,imgToProcess,erodeElement);
}

void LectureFeu::closing(cv::Mat &imgToProcess, cv::Size size, int shape)
{
    cv::Mat erodeElement = getStructuringElement(shape,size);
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = getStructuringElement(shape,size);

    erode(imgToProcess,imgToProcess,erodeElement);
    dilate(imgToProcess,imgToProcess,dilateElement);
}


/*==========  Parametrages (Acesseurs params HSV)  ==========*/

int LectureFeu::getHSV_max_value()
{
    int tmp;
    nh_.param<int>("feu_tricolore/HSV_threshold/value/max", tmp, 255);
    return tmp;
}

int LectureFeu::getHSV_min_value()
{
    int tmp;
    nh_.param<int>("feu_tricolore/HSV_threshold/value/min", tmp, 0);
    return tmp;
}

bool LectureFeu::getEnableHSV()
{
    bool tmp;
    nh_.param<bool>("feu_tricolore/HSV_threshold/enabled", tmp, true);
    return tmp;
}

int LectureFeu::getOpeningSize()
{
    int tmp;
    nh_.param<int>("feu_tricolore/opening/size", tmp, 0);
    return ((tmp<1)?1:tmp*2+1);
}

int LectureFeu::getOpeningIterations()
{
    int tmp;
    nh_.param<int>("feu_tricolore/opening/iteration", tmp, 0);
    return tmp;
}

bool LectureFeu::getEnableOpening()
{
    bool tmp;
    nh_.param<bool>("feu_tricolore/opening/enabled", tmp, true);
    return tmp;
}

int LectureFeu::getClosingSize()
{
    int tmp;
    nh_.param<int>("feu_tricolore/closing/size", tmp, 0);
    return ((tmp<1)?1:tmp*2+1);
}

int LectureFeu::getClosingIterations()
{
    int tmp;
    nh_.param<int>("feu_tricolore/closing/iteration", tmp, 0);
    return tmp;
}

bool LectureFeu::getEnableClosing()
{
    bool tmp;
    nh_.param<bool>("feu_tricolore/closing/enabled", tmp, true);
    return tmp;
}

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

cv::Mat LectureFeu::morphOps(cv::Mat imgBinary, char channel)
{
// Environement
    std::string paramBaseName;
    std::stringstream paramStream;
    std::string acceptedChannels("BGRHSV");
    cv::Mat binary = cv::Mat(imgBinary.rows, imgBinary.cols, CV_8UC(1), Scalar::all(255));

    // Prepare params recuperation
    if(acceptedChannels.find(channel) == std::string::npos)
    {
        ROS_WARN_STREAM("LectureFeu::morphOps : Unknown channel to threshold (Ch "<<channel<<")");
        return binary;
    }
    else
    {
        paramStream << "/trait_im/feu_tricolore/tmp/"<<channel<<"/morphops/";
        paramBaseName = paramStream.str();
    }

    // Recup params
    bool enableOpening;
    int openingIterations, openingSize;
    nh_.param<bool>(paramBaseName+"opening/enabled", enableOpening, true);
    nh_.param<int>(paramBaseName+"opening/iterations", openingIterations, 1);
    nh_.param<int>(paramBaseName+"opening/size", openingSize, 1);
    openingSize = ((openingSize<1)?1:openingSize*2+1);
    bool enableClosing;
    int closingIterations, closingSize;
    nh_.param<bool>(paramBaseName+"closing/enabled", enableClosing, true);
    nh_.param<int>(paramBaseName+"closing/iterations", closingIterations, 1);
    nh_.param<int>(paramBaseName+"closing/size", closingSize, 1);
    closingSize = ((closingSize<1)?1:closingSize*2+1);


// Algo
    // Opening - Closing - Opening on binary
    imgBinary.copyTo(binary);
    // Opening operation - 1
    if(enableOpening) {
        for(int k=0;k<openingIterations;k++) {
            opening(binary, cv::Size(openingSize,openingSize));
        }
    }

    // Closing operation -1
    if(enableClosing) {
        for(int k=0;k<closingIterations;k++) {
            closing(binary, cv::Size(closingSize,closingSize));
        }
    }

    // Openng operation- 2
    if(enableOpening) {
        for(int k=0;k<openingIterations;k++) {
            opening(binary, cv::Size(openingSize,openingSize));
        }
    }

    return binary;
}

cv::Mat LectureFeu::binaryMask(cv::Mat imgMultChannels, cv::Mat binaryMask)
{
// Environment
    cv::Mat result;
    cv::Mat binaryThreeChannels = singleToMultChannels(binaryMask, 3);

// Algo
    cv::bitwise_and(imgMultChannels, binaryThreeChannels, result);

    return result;
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


//     // Draw the histograms for H, S and V
//     int hist_w = 512; int hist_h = 400;
//     int bin_w = cvRound( (double) hist_w/histSize );

//     cv::Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

//     /// Normalize the result to [ 0, histImage.rows ]
//     normalize(h_hist, h_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//     normalize(s_hist, s_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//     normalize(v_hist, v_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

//     // // Compte les classes avant le filtrage
//     int green_pix= 0, red_pix = 0, yellow_pix;
//     // for( int i = 1; i < histSize; i++ )
//     // {
//     //     if(i < 50 || i >= 220)
//     //         red_pix += v_hist.at<float>(i);
//     //     else if(i >=140)
//     //         yellow_pix += v_hist.at<float>(i);
//     //     else
//     //         green_pix += v_hist.at<float>(i);
//     // }
//     // ROS_INFO_STREAM("Red : " << red_pix << " Yellow : " << yellow_pix << " Green : " << green_pix);

//     // // Filtrage de l'histo Hue
//     // #define INT_WIDTH 20
//     // float integral = 0;
//     // for( int i = 1; i < histSize; i++ )
//     // {
//     //     // Somme des 10 dernieres colonnes de l'histogram
//     //     integral += v_hist.at<float>(i);
//     //     if(i>=INT_WIDTH+1)
//     //     {
//     //         integral -= v_hist.at<float>(i-INT_WIDTH);
//     //         v_hist.at<float>(i-(INT_WIDTH/2)) = integral/INT_WIDTH;
//     //     }
//     // }

//     // Compte les classes apres le filtrage
//     green_pix= 0; red_pix = 0; yellow_pix = 0;
//     for( int i = 1; i < histSize; i++ )
//     {
//         if(i < 0 || i >= 235)
//             red_pix += hist[2].at<float>(i);
//         else if(i >=215)
//             yellow_pix += hist[2].at<float>(i);
//         else
//             green_pix += hist[2].at<float>(i);
//     }
//     ROS_INFO_STREAM("Red : " << red_pix << " Yellow : " << yellow_pix << " Green : " << green_pix);
//     int sum = red_pix + yellow_pix + green_pix;
//     if(sum!=0)
//     {
//         red_pix = red_pix*100/sum;
//         yellow_pix = yellow_pix*100/sum;
//         green_pix = green_pix*100/sum;
//     }
//     else
//     {
//         red_pix = yellow_pix = green_pix = 0;
//     }
//     ROS_INFO_STREAM(" --- Red : " << red_pix << "% Yellow : " << yellow_pix << "% Green : " << green_pix << "%");

//     /// Draw for each channel
//     for( int i = 1; i < histSize; i++ )
//     {
//       line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[2].at<float>(i-1)*hist_h/hmax[2]) ) ,
//                        Point( bin_w*(i), hist_h - cvRound(hist[2].at<float>(i)*hist_h/hmax[2]) ),
//                        Scalar( 0, 0, 255), 2, 8, 0  );
//       line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[1].at<float>(i-1)*hist_h/hmax[1]) ) ,
//                        Point( bin_w*(i), hist_h - cvRound(hist[1].at<float>(i)*hist_h/hmax[1]) ),
//                        Scalar( 255, 0, 0), 2, 8, 0  );
//       line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist[0].at<float>(i-1)*hist_h/hmax[0]) ) ,
//                        Point( bin_w*(i), hist_h - cvRound(hist[0].at<float>(i)*hist_h/hmax[0]) ),
//                        Scalar( 255, 255, 255), 2, 8, 0  );
//     }
/*-----  End of Class Definition  ------*/

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame )
{
  std::vector<Rect> lights;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect lights
  cascade.detectMultiScale( frame_gray, lights, 1.1, 2, 0, Size(40, 110));

  for( size_t i = 0; i < lights.size(); i++ )
  {
    rectangle( frame, lights[i], Scalar(rand()%255,rand()%255,rand()%255));
  }
  //-- Show what you got
  imshow( "Result", frame );
 }
