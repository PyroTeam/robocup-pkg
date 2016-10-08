#include <feu_tricolore/LightDetection.h>


namespace enc = sensor_msgs::image_encodings;


LightDetection::LightDetection()
: m_it(m_nh)
, m_action_name(ros::this_node::getName())
, m_as(m_nh, ros::this_node::getName(), false)
, m_inputColorInverted(m_nh, "computerVision/lightSignalDetection/inputColorInverted", false)
, m_roiParams(m_nh)
{
	// TODO: Paramétrer le topic d'image input ?

	// Ros topics
	m_image_sub = m_it.subscribe("hardware/camera/platform_camera/image_raw", 1, &LightDetection::imageCb, this);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/0_Flux_origine", m_image_sub.getTopic());

	m_result_pub = m_it.advertise("computerVision/lightSignalDetection/img_result", 1, true);
	m_nh.setParam("computerVision/lightSignalDetection/image_result/list/10_Image_resultat", m_result_pub.getTopic());

	ROS_INFO("Will do computer vision on following topic: %s", m_image_sub.getTopic().c_str());
	ROS_INFO("Will publish results on following topic: %s", m_result_pub.getTopic().c_str());

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

	m_feedback.percent_complete = (ros::Time::now()-m_beginOfProcessing).toSec()*100 / m_minProcessTimeNeeded;
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
	{
		  return;
	}

	/* Get image in BGR8
	 * OpenCV commonly expect BGR8 images
	 */
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		// XXX: Attention, common_utils/Parameter.h n'est pas templaté et ça craint.
		// Par exemple, pour l'instant il faut utiliser un paramètre floatant plutôt qu'un booléen
		if (!m_inputColorInverted())
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		else
		{
			// In some cases, input color could be inverted, so we need to trick cv_bridge in order
			// to obtain the good color channels
			cv_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);
			cv_ptr->encoding = enc::BGR8;
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv_ptr->image.copyTo(m_origin);
	m_inputImgHeader = msg->header;

	// Do the processing
	doLightDetection();
}

void LightDetection::initMembersImgs()
{
	// Images
	// TODO: Gérer la taille des images selon l'image d'entrée
	m_origin.create(240, 320, CV_8UC3);
	cv::randu(m_origin, cv::Scalar(0), cv::Scalar(256));
}

/*==========  Fonctions assurants la detection et la lecture du feu  ==========*/
void LightDetection::preTraitement(cv::Mat &imgToProcess)
{
	// Filtrage
	cv::blur(imgToProcess, imgToProcess, cv::Size(5,5));
}

void LightDetection::traitement(cv::Mat &imgToProcess)
{
	// TODO: Ajouter du ROS_LOG_NAMED
	constexpr float blinkTolerance = 0.5;
	constexpr float downBlinkTolerance = blinkTolerance/2;
	constexpr float topBlinkTolerance = blinkTolerance + downBlinkTolerance;
	constexpr int hsvValueChannel = 2;
	constexpr int greenValueThreshold  = 150;  // TODO: Ajouter un paramètre
	constexpr int yellowValueThreshold = 210;  // TODO: Ajouter un paramètre
	constexpr int redValueThreshold    = 210;  // TODO: Ajouter un paramètre

	float timeElapsed = (ros::Time::now() - m_beginOfProcessing).toSec();

	// Get hsv input image
    cv::Mat hsv;
    cv::cvtColor(imgToProcess, hsv, CV_BGR2HSV);

	// Get CV Regions of interest
	m_greenRoi = Rect(   std::floor(m_roiParams.green.xmin()*hsv.cols)
								, std::floor(m_roiParams.green.ymin()*hsv.rows)
								, std::ceil( m_roiParams.green.xmax()*hsv.cols - m_roiParams.green.xmin()*hsv.cols)
								, std::ceil( m_roiParams.green.ymax()*hsv.rows - m_roiParams.green.ymin()*hsv.rows));
	m_yellowRoi = Rect(  std::floor(m_roiParams.yellow.xmin()*hsv.cols)
								, std::floor(m_roiParams.yellow.ymin()*hsv.rows)
								, std::ceil( m_roiParams.yellow.xmax()*hsv.cols - m_roiParams.yellow.xmin()*hsv.cols)
								, std::ceil( m_roiParams.yellow.ymax()*hsv.rows - m_roiParams.yellow.ymin()*hsv.rows));
	m_redRoi = Rect(	  std::floor(m_roiParams.red.xmin()*hsv.cols)
								, std::floor(m_roiParams.red.ymin()*hsv.rows)
								, std::ceil( m_roiParams.red.xmax()*hsv.cols - m_roiParams.red.xmin()*hsv.cols)
								, std::ceil( m_roiParams.red.ymax()*hsv.rows - m_roiParams.red.ymin()*hsv.rows));

	cv::Mat greenLight = hsv(m_greenRoi);
	cv::Mat yellowLight = hsv(m_yellowRoi);
	cv::Mat redLight = hsv(m_redRoi);

	// Get mean value (luminosity) for each ROI and compare against configurable threshold
	float meanGreen = cv::mean(greenLight).val[hsvValueChannel];
	float meanYellow = cv::mean(yellowLight).val[hsvValueChannel];
	float meanRed = cv::mean(redLight).val[hsvValueChannel];

	m_greenTurnedOn = meanGreen > greenValueThreshold;
	m_yellowTurnedOn = meanYellow > yellowValueThreshold;
	m_redTurnedOn = meanRed > redValueThreshold;

	ROS_DEBUG_STREAM_NAMED("roi", "Mean values g: "<<meanGreen<<" y: "<<meanYellow<<" r: "<<meanRed);

	// Save results
	m_nbRedTurnedOn 	+= ((m_redTurnedOn)		? 1 : 0);
	m_nbYellowTurnedOn	+= ((m_yellowTurnedOn)	? 1 : 0);
	m_nbGreenTurnedOn	+= ((m_greenTurnedOn)	? 1 : 0);

	// After a certain time, process result (check if blinking or not)
	if(timeElapsed >= m_minProcessTimeNeeded)
	{
		// Action feedback
		m_feedback.percent_complete = 100;
		m_feedback.images_processed = m_nbImgProcessed;
		m_as.publishFeedback(m_feedback);

		// A result is only valid if we have processed enough images
		if (m_nbImgProcessed/timeElapsed >= m_minNbImgProcessedPerSecond)
		{
			// Action result
			comm_msg::LightSpec light;

			light.color = light.RED;
			if(m_nbRedTurnedOn > m_nbImgProcessed*topBlinkTolerance)
			{
				light.state = light.ON;
			}
			else if(m_nbRedTurnedOn < m_nbImgProcessed*downBlinkTolerance)
			{
				light.state = light.OFF;
			}
			else
			{
				light.state = light.BLINK;
			}
			m_finalRedSignal = light.state;
			m_result.light_signal.push_back(light);

			light.color = light.YELLOW;
			if(m_nbYellowTurnedOn > m_nbImgProcessed*topBlinkTolerance)
			{
				light.state = light.ON;
			}
			else if(m_nbYellowTurnedOn < m_nbImgProcessed*downBlinkTolerance)
			{
				light.state = light.OFF;
			}
			else
			{
				light.state = light.BLINK;
			}
			m_finalYellowSignal = light.state;
			m_result.light_signal.push_back(light);

			light.color = light.GREEN;
			if(m_nbGreenTurnedOn > m_nbImgProcessed*topBlinkTolerance)
			{
				light.state = light.ON;
			}
			else if(m_nbGreenTurnedOn < m_nbImgProcessed*downBlinkTolerance)
			{
				light.state = light.OFF;
			}
			else
			{
				light.state = light.BLINK;
			}
			m_finalGreenSignal = light.state;
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

static void drawSignalOnImg(cv::Mat &img, cv::Rect &roi, int signal)
{
	constexpr int maxValue = 255;
	static const cv::Scalar blackColor = cv::Scalar(0.0*maxValue, 0.0*maxValue, 0.0*maxValue);
	static const cv::Scalar whiteColor = cv::Scalar(1.0*maxValue, 1.0*maxValue, 1.0*maxValue);
	static const cv::Scalar blueColor = cv::Scalar(1.0*maxValue, 0.0*maxValue, 0.0*maxValue);

	constexpr int lineThickness = 5;
	constexpr int lineOffset = lineThickness - lineThickness/2;

	const cv::Scalar *color;

	switch (signal)
	{
		case comm_msg::LightSpec::ON:
			color = &whiteColor;
		break;

		case comm_msg::LightSpec::OFF:
			color = &blackColor;
		break;

		case comm_msg::LightSpec::BLINK:
			color = &blueColor;
		break;

		default:
			ROS_ERROR("Unkown signal: %d", signal);
			return;
	}

	cv::line(img
			, cv::Point(roi.br().x+lineOffset, roi.tl().y)
			, cv::Point(roi.br().x+lineOffset, roi.br().y)
			, *color, lineThickness);
}

void LightDetection::publishResultImages(cv::Mat &imgResult)
{
	// TODO: Ajouter à la lib common, une fonction permettant, à partir d'une cv::Mat ou de
	// cv::Mat::depth de retrouver la valeur max possible pour cette dernière
	constexpr int maxValue = 255;
	static const cv::Scalar greenColor = cv::Scalar(0.0*maxValue, 1.0*maxValue, 0.0*maxValue);
	static const cv::Scalar yellowColor = cv::Scalar(0.1*maxValue, 0.5*maxValue, 1.0*maxValue);
	static const cv::Scalar redColor = cv::Scalar(0.0*maxValue, 0.0*maxValue, 1.0*maxValue);
	static const cv::Scalar blackColor = cv::Scalar(0.0*maxValue, 0.0*maxValue, 0.0*maxValue);
	constexpr int lineThickness = 5;
	constexpr int lineOffset = lineThickness - lineThickness/2;

	// Dessine les ROI sur l'image
	cv::rectangle(imgResult, m_greenRoi,  greenColor);
	cv::rectangle(imgResult, m_yellowRoi, yellowColor);
	cv::rectangle(imgResult, m_redRoi,    redColor);

	// Dessine l'interprétation des couleurs
	// Trois lignes, noire si feu éteint, ou de la couleur du feu si feu allumé
	// Les lignes font 5pixels de larges et sont juste à gauche des ROIs
	cv::line(imgResult
			, cv::Point(m_greenRoi.tl().x-lineOffset, m_greenRoi.tl().y)
			, cv::Point(m_greenRoi.tl().x-lineOffset, m_greenRoi.br().y)
			, m_greenTurnedOn? greenColor : blackColor, lineThickness);
	cv::line(imgResult
			, cv::Point(m_yellowRoi.tl().x-lineOffset, m_yellowRoi.tl().y)
			, cv::Point(m_yellowRoi.tl().x-lineOffset, m_yellowRoi.br().y)
			, m_yellowTurnedOn? yellowColor : blackColor, lineThickness);
	cv::line(imgResult
			, cv::Point(m_redRoi.tl().x-lineOffset, m_redRoi.tl().y)
			, cv::Point(m_redRoi.tl().x-lineOffset, m_redRoi.br().y)
			, m_redTurnedOn? redColor : blackColor, lineThickness);

	// TODO: Enrichir le result de l'action et verifier si on a réussi ou échouer
	// Si l'action est terminée, ajoute des informations
	if (!m_as.isActive())
	{
		// Noir pour éteint. Blanc pour allumé. Bleu pour clignotant.
		drawSignalOnImg(imgResult, m_greenRoi, m_finalGreenSignal);
		drawSignalOnImg(imgResult, m_yellowRoi, m_finalYellowSignal);
		drawSignalOnImg(imgResult, m_redRoi, m_finalRedSignal);
	}

	// Publication des images
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(m_inputImgHeader, enc::BGR8, imgResult).toImageMsg();
	m_result_pub.publish(msg);
}

void LightDetection::doLightDetection(cv::Mat &imgToProcess)
{
	++m_nbImgProcessed;

	preTraitement(imgToProcess);
	traitement(imgToProcess);
	// TODO: Ajouter un paramètre pour désactiver cette partie
	publishResultImages(imgToProcess);
}
