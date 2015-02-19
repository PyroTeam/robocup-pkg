/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>

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


    if( !cascade.load( cascade_name ) ){ printf("--(!)Error loading\n"); return; };
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

    // templateProcessing();
    // hsvProcessing();
    // featureProcessing();
    // freakProcessing();

    // std::vector<std::string> detectors;
    // std::vector<std::string> extractors;
    // std::vector<std::string> matchers;

    // detectors.push_back("FAST");
    // detectors.push_back("STAR");
    // detectors.push_back("ORB");
    // detectors.push_back("BRISK");
    // detectors.push_back("MSER");
    // detectors.push_back("GFTT");
    // detectors.push_back("HARRIS");
    // detectors.push_back("Dense");
    // detectors.push_back("SimpleBlob");

    // extractors.push_back("BRIEF");
    // extractors.push_back("BRISK");
    // extractors.push_back("ORB");
    // extractors.push_back("FREAK");     

    // // matchers.push_back("BruteForce");
    // // matchers.push_back("BruteForce-L1");
    // // matchers.push_back("BruteForce-Hamming");
    // matchers.push_back("BruteForce-Hamming(2)");
    // // matchers.push_back("FlannBased");

    // if(!processed)
    // {
    //     for (int i = 0; i < detectors.size(); ++i)
    //     {
    //         ROS_INFO("\n=================\n\n=================\n\n=================");
    //         for (int j = 0; j < extractors.size(); ++j)
    //         {
    //             for (int k = 0; k < matchers.size(); ++k)
    //             {
    //                 freakProcessing(detectors[i],extractors[j],matchers[k]);
    //             }
    //         }
    //     }
    // }
    if(!processed || 1)
    {
        // freakProcessing();
        cascadeProcessing();
    }
}

void LectureFeu::hsvProcessing()
{
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

        nh_.param<int>("feu_tricolore/HSV_threshold/value/max", HSV_max_value, 255);
        nh_.param<int>("feu_tricolore/HSV_threshold/value/min", HSV_min_value, 0);
        nh_.param<bool>("feu_tricolore/HSV_threshold/enabled", enableHSV, true);
        
        nh_.param<int>("feu_tricolore/opening/size", openingSize, 0);
        nh_.param<int>("feu_tricolore/opening/iteration", openingIterations, 0);
        nh_.param<bool>("feu_tricolore/opening/enabled", enableOpening, true);

        nh_.param<int>("feu_tricolore/closing/size", closingSize, 0);
        nh_.param<int>("feu_tricolore/closing/iteration", closingIterations, 0);
        nh_.param<bool>("feu_tricolore/closing/enabled", enableClosing, true);
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

    cv::Mat erodeElement;
    cv::Mat dilateElement;

// Opération d'ouverture - 1
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

// Opération de fermeture -1
    erodeElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); // ancien 3.3
    //dilate with larger element so make sure object is nicely visible
    dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); //ancien 3.3 puis 8.8

    if(enableClosing)
    {
        for(int k=0;k<closingIterations;k++){
            erode(result_threshold,result_threshold,erodeElement);
            dilate(result_threshold,result_threshold,dilateElement);
        }        
    }

// Opération d'ouverture - 2
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

// // Opération de fermeture - 2
//     erodeElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); // ancien 3.3
//     //dilate with larger element so make sure object is nicely visible
//     dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(closingSize,closingSize)); //ancien 3.3 puis 8.8

//     if(enableClosing)
//     {
//         for(int k=0;k<closingIterations;k++){
//             erode(result_threshold,result_threshold,erodeElement);
//             dilate(result_threshold,result_threshold,dilateElement);
//         }        
//     }

    cv::bitwise_and(_origin_rgb, result_threshold, _result);
// Publication des images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _result).toImageMsg();
    result_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _hsv).toImageMsg();
    hsv_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _thesholded).toImageMsg();
    before_morphops_pub_.publish(msg);
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
        if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        { 
            matchLoc = minLoc; 
            if(minMax == -1 || minVal < minMax)
            {
                minMax = minVal;
                true_matchLoc = matchLoc;
            }
        }
        else
        { 
            matchLoc = maxLoc;
            if(minMax == -1 || maxVal > minMax)
            {
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
    hsv_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, _thesholded).toImageMsg();
    before_morphops_pub_.publish(msg);
}

int LectureFeu::featureProcessing()
{
    std::string base_dir("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/img/joao_pessoa/");
    cv::Mat img_1 = imread( base_dir + "TL_templ_001.jpg", CV_LOAD_IMAGE_COLOR);
    // cv::Mat img_1 = imread( base_dir + "000.jpg", CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2;
    _origin.copyTo(img_2);

    if( !img_1.data || !img_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

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
    hsv_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, img_1).toImageMsg();
    before_morphops_pub_.publish(msg);
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
    hsv_pub_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, imgKeypointB).toImageMsg();
    before_morphops_pub_.publish(msg);

    processed = true;

    return 0;
}

int LectureFeu::cascadeProcessing()
{
    detectAndDisplay(_origin);

    processed = true;

    return 0;
}

bool LectureFeu::ok()
{
    cv::waitKey(1);
    return nh_.ok();
}

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