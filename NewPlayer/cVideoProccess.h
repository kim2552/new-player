#pragma once

#include "wx/wx.h"

// OpenCV libraries
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

class cVideoProccess
{
public:
	cVideoProccess();
	~cVideoProccess();

public:
	void ProcessVideo();
};

