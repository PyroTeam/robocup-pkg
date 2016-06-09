/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>


/*========================================
=            Class Definition            =
========================================*/

LectureFeu::LectureFeu()
: m_it(m_nh)
, m_action_name(ros::this_node::getName())
, m_as(m_nh, ros::this_node::getName(), false)
{
	// Ros topics
	m_image_sub = m_it.subscribe("hardware/camera/platform_camera/image_raw", 1, &LectureFeu::imageCb, this);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/0_Flux_origine", "/image_raw");

	m_result_pub = m_it.advertise("computerVision/lightSignalDetection/img_result", 1);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/10_Image_resultat", "computerVision/lightSignalDetection/img_result");

	// Init members cv::Mat
	initMembersImgs();

	// Ros action server
	m_as.registerGoalCallback(boost::bind(&LectureFeu::goalCB, this));
	m_as.registerPreemptCallback(boost::bind(&LectureFeu::preemptCB, this));
	m_as.start();
}

void LectureFeu::goalCB()
{
	// TODO: Sligthly change Action definition, and add ROI parameters in it.
	//
	// Cela permetra une utilisation plus souple de la détection d'image.
	// L'executeur de tâche au moment de faire la requête doit immobiliser le robot et ce jusqu'à
	// ce que la requête soit terminée. Idéalement, le feu devrait toujours être au même endroit dans l'image.
	// Mais supposons qu'il soit placé différemment sur une machine
	// ou que le robot soit placé un peu moins précisément qu'attendu.
	// L'executeur pouvant déteminer où se trouve le feu sur la machine et où se trouve le robot
	// par rapport à la machine (le tout grâce à l'arTag visible). L'executeur pourra alors demander
	// une détection de feu avec une ROI plus adapté à l'image réelle du feu.

	// Accept the new goal
	m_as.acceptNewGoal();

	// Re-init
	m_nbImgProcessed = 0;
	m_nbRedTurnedOn = 0;
	m_nbYellowTurnedOn = 0;
	m_nbGreenTurnedOn = 0;


	// First Action Feedback
	m_beginOfProcessing = ros::Time::now();
	m_nbImgProcessed = 0;

	m_feedback.percent_complete = (ros::Time::now()-m_beginOfProcessing).toSec() / m_minProcessTimeNeeded;
	m_as.publishFeedback(m_feedback);
}

void LectureFeu::preemptCB()
{
	ROS_INFO("%s: Preempted", m_action_name.c_str());
	m_as.setPreempted();

	// Re-init members cv::Mat
	initMembersImgs();
}

LectureFeu::~LectureFeu()
{
	m_image_sub.shutdown();
	m_result_pub.shutdown();
}

bool LectureFeu::ok()
{
	cv::waitKey(1);
	return m_nh.ok();
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
	if (!m_as.isActive())
		  return;

	// Get image in BGR8
	// TODO: Add a special parameter for handling bad encoded images, i.e. ignoring camera_info encoding
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
	cv_ptr->image.copyTo(m_origin);

	// TODO: See above : "Add a special parameter for handling bad encoded images, i.e. ignoring camera_info encoding"
	cv::cvtColor(m_origin,m_origin_rgb,CV_BGR2RGB);

	// Do the processing
	float timeElapsed = (ros::Time::now()-m_beginOfProcessing).toSec();
	++m_nbImgProcessed;
	lectureFeu();

	// TODO: Move all below on specific function
	// Save results
	m_nbRedTurnedOn+=((m_redTurnedOn)?1:0);
	m_nbYellowTurnedOn+=((m_yellowTurnedOn)?1:0);
	m_nbGreenTurnedOn+=((m_greenTurnedOn)?1:0);

	// TODO: Manage time with topic time
	if(timeElapsed >= m_minProcessTimeNeeded)
	{
		// Action feedback
		m_feedback.percent_complete = 100;
		m_feedback.images_processed = m_nbImgProcessed;
		m_as.publishFeedback(m_feedback);

		// Good result only if we have enough images processed
		if (m_nbImgProcessed/timeElapsed >= m_minNbImgProcessedPerSecond)
		{
			// Action result
			comm_msg::LightSpec light;

			light.color = light.RED;
			if(m_nbRedTurnedOn > m_nbImgProcessed*0.66)
				light.state = light.ON;
			else if(m_nbRedTurnedOn < m_nbImgProcessed*0.3)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			light.color = light.YELLOW;
			if(m_nbYellowTurnedOn > m_nbImgProcessed*0.66)
				light.state = light.ON;
			else if(m_nbYellowTurnedOn < m_nbImgProcessed*0.3)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			light.color = light.GREEN;
			if(m_nbGreenTurnedOn > m_nbImgProcessed*0.66)
				light.state = light.ON;
			else if(m_nbGreenTurnedOn < m_nbImgProcessed*0.3)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			// set the action state to succeeded
			m_as.setSucceeded(m_result);

			return;
		}
		// Action result - all off because of abortion
		comm_msg::LightSpec light;

		light.color = light.RED;
		light.state = light.OFF;
		m_result.light_signal.push_back(light);

		light.color = light.YELLOW;
		light.state = light.OFF;
		m_result.light_signal.push_back(light);

		light.color = light.GREEN;
		light.state = light.OFF;
		m_result.light_signal.push_back(light);

			//set the action state to aborted
		m_as.setAborted(m_result);

		return;
	}
	else {
	// Action Feedback
		m_feedback.percent_complete = (timeElapsed*100)/m_minProcessTimeNeeded;
		// m_feedback.percent_complete = timeElapsed;
		m_feedback.images_processed = m_nbImgProcessed;
		m_as.publishFeedback(m_feedback);
	}
}

void LectureFeu::initMembersImgs()
{
	// Images
	m_origin.create(240, 320, CV_8UC3);
	cv::randu(m_origin, cv::Scalar(0), cv::Scalar(256));
	m_origin_treated.create(240, 320, CV_8UC3);
	cv::randu(m_origin_treated, cv::Scalar(0), cv::Scalar(256));
	m_origin_rgb.create(240, 320, CV_8UC3);
	cv::randu(m_origin_rgb, cv::Scalar(0), cv::Scalar(256));
	m_hsv.create(240, 320, CV_8UC3);
	cv::randu(m_hsv, cv::Scalar(0), cv::Scalar(256));
	m_thesholded.create(240, 320, CV_8UC3);
	cv::randu(m_thesholded, cv::Scalar(0), cv::Scalar(256));

	m_origin.create(240, 320, CV_8UC3);
	cv::randu(m_origin, cv::Scalar(0), cv::Scalar(256));
	m_output_1.create(240, 320, CV_8UC3);
	cv::randu(m_output_1, cv::Scalar(0), cv::Scalar(256));
	m_output_2.create(240, 320, CV_8UC3);
	cv::randu(m_output_2, cv::Scalar(0), cv::Scalar(256));
	m_output_3.create(240, 320, CV_8UC3);
	cv::randu(m_output_3, cv::Scalar(0), cv::Scalar(256));
	m_output_4.create(240, 320, CV_8UC3);
	cv::randu(m_output_4, cv::Scalar(0), cv::Scalar(256));
	m_output_5.create(240, 320, CV_8UC3);
	cv::randu(m_output_5, cv::Scalar(0), cv::Scalar(256));
	m_resultimg.create(240, 320, CV_8UC3);
	cv::randu(m_resultimg, cv::Scalar(0), cv::Scalar(256));
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
	/*
		cv::Mat image = imread("");
		cv::Rect region_of_interest = Rect(x, y, w, h);
		cv::Mat image_roi = image(region_of_interest);
	*/
}

void LectureFeu::publishResults(cv::Mat &imgToProcess)
{
	// Publication des images
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, imgToProcess).toImageMsg();
	m_result_pub.publish(msg);
}

void LectureFeu::lectureFeu(cv::Mat &imgToProcess)
{
	m_redTurnedOn = false;
	m_yellowTurnedOn = false;
	m_greenTurnedOn = false;

	preTraitement(imgToProcess);
	traitement(imgToProcess);
	publishResults(imgToProcess);
}


/*==========  Utils  ==========*/

cv::Mat singleToMultChannels(cv::Mat binary, int numChannels)
{
	// Add copies of binary img in a tab
	std::vector<cv::Mat> binTab(numChannels);
	for(int i = 0;i<numChannels;++i) { binTab[i] = binary; }

	// Merge them in one img
	cv::Mat imgMultChannels;
	cv::merge(binTab, imgMultChannels);

	return imgMultChannels;
}


cv::Mat calcHist(cv::Mat imgToHist, int channel, bool normalize)
{
	if(channel > imgToHist.channels())
	{
		ROS_ERROR_STREAM("LectureFeu::calcHist : Requested channel "<<channel<<" on a "
							<<imgToHist.channels()<<" channel(s) image");
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


cv::Mat histToImg(cv::Mat hist)
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
		// TODO: Paramétrer ces plages de couleurs
		// Rouge
		if(i < 10 || (i >= 170 && i <= 180))
		line( histImage, Point( bin_w*(i), hist_h ) ,
					   Point( bin_w*(i), 0 ),
					   Scalar( 0, 0, 50), 2, 8, 0  );
		// Vert
		if(i < 90 && i >= 70)
		line( histImage, Point( bin_w*(i), hist_h ) ,
					   Point( bin_w*(i), 0 ),
					   Scalar( 0, 50, 0), 2, 8, 0  );
		// Jaune
		if(i < 30 && i >= 10)
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