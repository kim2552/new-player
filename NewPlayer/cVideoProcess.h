#pragma once

#include "globals.h"

#include "wx/wx.h"

// OpenCV libraries
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

class cVideoProcess
{
public:
	cVideoProcess();
	~cVideoProcess();

public:
	void ProcessFrame(cv::Mat& frame);
	void ProcessVideo();
};

