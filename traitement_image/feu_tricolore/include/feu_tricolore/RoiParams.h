#ifndef _TRAITEMENT_IMAGE__ROI_PARAMS__H_
#define _TRAITEMENT_IMAGE__ROI_PARAMS__H_

#include <common_utils/Parameter.h>

struct oneRoiParams_t
{
	Parameter xmin;
	Parameter xmax;
	Parameter ymin;
	Parameter ymax;

	oneRoiParams_t(ros::NodeHandle &nh, const char *xmi, const char *xma, const char *ymi, const char *yma)
	: xmin(nh, xmi, 0.0)
	, xmax(nh, xma, 1.0)
	, ymin(nh, ymi, 0.0)
	, ymax(nh, yma, 1.0)
	{}
};

struct RoiParams
{
	RoiParams(ros::NodeHandle &nh)
	: green(nh
		, "computerVision/lightSignalDetection/greenRoi/xmin"
		, "computerVision/lightSignalDetection/greenRoi/xmax"
		, "computerVision/lightSignalDetection/greenRoi/ymin"
		, "computerVision/lightSignalDetection/greenRoi/ymax")
	, yellow(nh
		, "computerVision/lightSignalDetection/yellowRoi/xmin"
		, "computerVision/lightSignalDetection/yellowRoi/xmax"
		, "computerVision/lightSignalDetection/yellowRoi/ymin"
		, "computerVision/lightSignalDetection/yellowRoi/ymax")
	, red(nh
		, "computerVision/lightSignalDetection/redRoi/xmin"
		, "computerVision/lightSignalDetection/redRoi/xmax"
		, "computerVision/lightSignalDetection/redRoi/ymin"
		, "computerVision/lightSignalDetection/redRoi/ymax")
	{
	}
	~RoiParams(){}

	oneRoiParams_t green;
	oneRoiParams_t yellow;
	oneRoiParams_t red;
};

#endif // _TRAITEMENT_IMAGE__ROI_PARAMS__H_