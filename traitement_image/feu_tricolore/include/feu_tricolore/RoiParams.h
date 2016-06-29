#ifndef _TRAITEMENT_IMAGE__ROI_PARAMS__H_
#define _TRAITEMENT_IMAGE__ROI_PARAMS__H_

#include <common_utils/Parameter.h>

struct oneRoiParams_t
{
	Parameter xmin;
	Parameter xmax;
	Parameter ymin;
	Parameter ymax;
	Parameter threshold;

	oneRoiParams_t(ros::NodeHandle &nh, const char *xmi, const char *xma, const char *ymi, const char *yma
									  , const char *thresh)
	: xmin(nh, xmi, 0.0)
	, xmax(nh, xma, 1.0)
	, ymin(nh, ymi, 0.0)
	, ymax(nh, yma, 1.0)
	, threshold(nh, thresh, 210)
	{}
};

struct RoiParams
{
	RoiParams(ros::NodeHandle &nh)
	: green(nh
		, "computerVision/lightSignalDetection/greenRoi/xmin"
		, "computerVision/lightSignalDetection/greenRoi/xmax"
		, "computerVision/lightSignalDetection/greenRoi/ymin"
		, "computerVision/lightSignalDetection/greenRoi/ymax"
		, "computerVision/lightSignalDetection/greenRoi/threshold")
	, yellow(nh
		, "computerVision/lightSignalDetection/yellowRoi/xmin"
		, "computerVision/lightSignalDetection/yellowRoi/xmax"
		, "computerVision/lightSignalDetection/yellowRoi/ymin"
		, "computerVision/lightSignalDetection/yellowRoi/ymax"
		, "computerVision/lightSignalDetection/yellowRoi/threshold")
	, red(nh
		, "computerVision/lightSignalDetection/redRoi/xmin"
		, "computerVision/lightSignalDetection/redRoi/xmax"
		, "computerVision/lightSignalDetection/redRoi/ymin"
		, "computerVision/lightSignalDetection/redRoi/ymax"
		, "computerVision/lightSignalDetection/redRoi/threshold")
	{
	}
	~RoiParams(){}

	oneRoiParams_t green;
	oneRoiParams_t yellow;
	oneRoiParams_t red;
};

#endif // _TRAITEMENT_IMAGE__ROI_PARAMS__H_