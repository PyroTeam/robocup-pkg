/*==========  Inludes  ==========*/
#include <feu_tricolore/SimLightDetection.h>

/*========================================
=            Class Definition            =
========================================*/

SimLightDetection::SimLightDetection():
	m_as(m_nh, "computerVision/lecture_feu", false),
	m_actionName("computerVision/lecture_feu")
{
	// Ros topics
	m_closestLightSignal_sub = m_nh.subscribe("hardware/closest_light_signal", 1, &SimLightDetection::closestLightSignal_callback, this);

	// Ros action server
	m_as.registerGoalCallback(boost::bind(&SimLightDetection::goalCB, this));
	m_as.registerPreemptCallback(boost::bind(&SimLightDetection::preemptCB, this));
	m_as.start();
}

void SimLightDetection::goalCB()
{
	m_as.acceptNewGoal();
	m_beginOfProcessing = ros::Time::now();

	// Action Feedback
	float timeElapsed = ros::Time::now().toSec() - m_beginOfProcessing.toSec();
	m_feedback.percent_complete = timeElapsed*100.0 / m_timeout;
	m_feedback.images_processed = 0;
	m_as.publishFeedback(m_feedback);

	// If a light signal exist, set action as succeeded
	if (m_result.light_signal.size() > 0)
	{
		m_as.setSucceeded(m_result);
	}
}

void SimLightDetection::preemptCB()
{
	ROS_INFO("%s: Preempted", m_actionName.c_str());
	// Set the action state to preempted
	m_as.setPreempted();
}

SimLightDetection::~SimLightDetection()
{
}

bool SimLightDetection::ok()
{
	// Update action feedback if action is active
	if (m_as.isActive())
	{
		float timeElapsed = ros::Time::now().toSec() - m_beginOfProcessing.toSec();
		// If the timeout is reached, abort action
		if(timeElapsed >= m_timeout)
		{
			// Action feedback
			float timeoutPercent = timeElapsed / m_timeout;
			m_feedback.percent_complete = (timeoutPercent > 1.0)?100:timeoutPercent*100.0;
			m_as.publishFeedback(m_feedback);

			// Set the action state to aborted
			m_result.light_signal.clear();
			m_as.setAborted(m_result);
		
		}
		else
		{
			// Action feedback
			m_feedback.percent_complete = timeElapsed*100.0 / m_timeout;
			m_as.publishFeedback(m_feedback);
		}
	}

	return m_nh.ok();
}

/*! \brief Closet light signal callback fonction
 * 
 * \details Get the closest light signal sent from simulation 
 *
 * \param[in] msg
 */
void SimLightDetection::closestLightSignal_callback(const trait_im_msg::LightSignalConstPtr &msg)
{
	// Save light signal
	m_result.light_signal = msg->lights;

	// If active action running and a light signal exist, set action as succeeded
	if (m_as.isActive() && m_result.light_signal.size() > 0)
	{
		// Action feedback
		float timeElapsed = ros::Time::now().toSec() - m_beginOfProcessing.toSec();
		m_feedback.percent_complete = timeElapsed*100.0 / m_timeout;
		m_as.publishFeedback(m_feedback);

		m_as.setSucceeded(m_result);
	}
}