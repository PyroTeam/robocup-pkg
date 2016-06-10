#include <feu_tricolore/LightDetection.h>


namespace enc = sensor_msgs::image_encodings;


LightDetection::LightDetection()
: m_it(m_nh)
, m_action_name(ros::this_node::getName())
, m_as(m_nh, ros::this_node::getName(), false)
{
	// Ros topics
	m_image_sub = m_it.subscribe("hardware/camera/platform_camera/image_raw", 1, &LightDetection::imageCb, this);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/0_Flux_origine", "/image_raw");

	m_result_pub = m_it.advertise("computerVision/lightSignalDetection/img_result", 1);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/10_Image_resultat", "computerVision/lightSignalDetection/img_result");

	// Init members cv::Mat
	initMembersImgs();

	// Ros action server
	m_as.registerGoalCallback(boost::bind(&LightDetection::goalCB, this));
	m_as.registerPreemptCallback(boost::bind(&LightDetection::preemptCB, this));
	m_as.start();
}

void LightDetection::goalCB()
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

	m_feedback.percent_complete = (ros::Time::now()-m_beginOfProcessing).toSec() / m_minProcessTimeNeeded;
	m_as.publishFeedback(m_feedback);
}

void LightDetection::preemptCB()
{
	ROS_INFO("%s: Preempted", m_action_name.c_str());
	m_as.setPreempted();

	// Re-init members cv::Mat
	initMembersImgs();
}

LightDetection::~LightDetection()
{
	m_image_sub.shutdown();
	m_result_pub.shutdown();
}

bool LightDetection::ok()
{
	return m_nh.ok();
}

void LightDetection::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
	doLightDetection();
}

void LightDetection::initMembersImgs()
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
void LightDetection::preTraitement(cv::Mat &imgToProcess)
{
	// Filtrage
	cv::blur(imgToProcess, imgToProcess, cv::Size(5,5));
}

void LightDetection::traitement(cv::Mat &imgToProcess)
{
	float timeElapsed = (ros::Time::now() - m_beginOfProcessing).toSec();


	// TODO: DO ROI TREATMENT
	/*
		cv::Mat image = imread("");
		cv::Rect region_of_interest = Rect(x, y, w, h);
		cv::Mat image_roi = image(region_of_interest);
	*/

	// Save results
	m_nbRedTurnedOn 	+= ((m_redTurnedOn)		? 1 : 0);
	m_nbYellowTurnedOn	+= ((m_yellowTurnedOn)	? 1 : 0);
	m_nbGreenTurnedOn	+= ((m_greenTurnedOn)	? 1 : 0);

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
			else if(m_nbRedTurnedOn < m_nbImgProcessed*0.33)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			light.color = light.YELLOW;
			if(m_nbYellowTurnedOn > m_nbImgProcessed*0.66)
				light.state = light.ON;
			else if(m_nbYellowTurnedOn < m_nbImgProcessed*0.33)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			light.color = light.GREEN;
			if(m_nbGreenTurnedOn > m_nbImgProcessed*0.66)
				light.state = light.ON;
			else if(m_nbGreenTurnedOn < m_nbImgProcessed*0.33)
				light.state = light.OFF;
			else
				light.state = light.BLINK;
			m_result.light_signal.push_back(light);

			// Set the action state to succeeded
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

		// Set the action state to aborted
		m_as.setAborted(m_result);

		return;
	}
	else
	{
		// Action Feedback
		m_feedback.percent_complete = (timeElapsed*100)/m_minProcessTimeNeeded;
		m_feedback.images_processed = m_nbImgProcessed;
		m_as.publishFeedback(m_feedback);
	}
}

void LightDetection::publishResultImages(cv::Mat &imgToProcess)
{
	// Publication des images
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, imgToProcess).toImageMsg();
	m_result_pub.publish(msg);
}

void LightDetection::doLightDetection(cv::Mat &imgToProcess)
{
	++m_nbImgProcessed;

	preTraitement(imgToProcess);
	traitement(imgToProcess);
	publishResultImages(imgToProcess);
}
