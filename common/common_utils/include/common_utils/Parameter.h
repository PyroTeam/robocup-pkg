/**
 * \file
 *
 * \brief		Parameter class
 *
 * \author      Valentin Vergez (valentin.vergez@gmail.com)
 * \date        2016-02-20
 * \copyright   PyroTeam, Polytech-Lille
 * \license		LGPLv3
 * \version
 */

#include <string>

#include <ros/ros.h>

// TODO: Document
typedef enum parameterCacheMode_e
{
	FORCE_CACHE, /*!< parameter will always be cached, even if refresh() is called */
	FORCE_REFRESH, /*!< parameter will always be refreshed, no need to call refresh() here */ 
	CONFIGURABLE_CACHE, /*!< parameter actualisation is */
	CONFIGURABLE_REFRESH,
	
} parameterCacheMode_t;

class Parameter
{
public:
	Parameter(ros::NodeHandle nh, std::string fullName);
	~Parameter(void);
	float get(void);
	float operator()(void);
	void refresh(void);
	void setMode(parameterCacheMode_t mode);
	parameterCacheMode_t getMode(void);

private:
	ros::NodeHandle m_nh;
	float m_value;
	std::string m_fullName;
	parameterCacheMode_t m_mode;	
};