#pragma once

#include "globals.h"

#include "wx/wx.h"

// OpenCV libraries
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

// dlib libraries
#include <dlib/opencv.h>
#include <dlib/image_io.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing/shape_predictor.h>

class cVideoProcess
{
public:
	cVideoProcess();
	~cVideoProcess();

public:
	void ProcessFrame(cv::Mat& src_frame, cv::Mat& dst_frame);
	void ProcessVideo();

private:
	dlib::frontal_face_detector ff_detector;	// To get bounding boxes for each face
	dlib::shape_predictor shape_pred;			// Predict face landmark predictions given image & face bounding box

#if DEVELOP_MODE
	dlib::image_window win;
#endif

};

