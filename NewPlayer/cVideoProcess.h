#pragma once

#include "globals.h"

#include "wx/wx.h"

// OpenCV libraries
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/photo.hpp"

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
	std::vector<std::vector<cv::Point>> FrontalFaceLandmarks(cv::Mat& src);
	std::vector<std::vector<cv::Point>> HullIndicies(std::vector<std::vector<cv::Point>> contours);

	cv::Mat TriangleMask(cv::Rect rectangle, std::vector<cv::Point> triangle);
	cv::Mat WarpTriangle(cv::Mat img_triangle, std::vector<cv::Point> src_triangle, std::vector<cv::Point> dst_triangle, cv::Rect src_rect, cv::Rect dst_rect, cv::Mat mask);
	void ReconstructFace(cv::Mat& img, cv::Rect boundingRect, cv::Mat warped_triangle);

	void DelaunayTriangulation(cv::Mat& img, cv::Subdiv2D& subdiv, std::vector<cv::Point> points, cv::Scalar delaunay_color);
	void ProcessSourceFrame(cv::Mat& src_frame, cv::Mat& dst_frame, cv::Subdiv2D& subdiv, std::vector<cv::VideoWriter> videos);
	void ProcessDestinationFrame(cv::Mat& src_frame, cv::Mat& dst_frame, cv::Subdiv2D& subdiv, std::vector<cv::VideoWriter> videos);
	void ProcessTriangulation(cv::Mat& img, cv::Mat& frame, std::vector<std::vector<cv::Point>>& hullIndices);
	void ProcessVideo();

private:
	dlib::frontal_face_detector ff_detector;	// To get bounding boxes for each face
	dlib::shape_predictor shape_pred;			// Predict face landmark predictions given image & face bounding box

	cv::Subdiv2D image_subdiv;
	cv::Subdiv2D frame_subdiv;

	std::vector<cv::Point> image_landmark_points;
	std::vector<cv::Point> frame_landmark_points;

	std::vector<std::vector<cv::Point>> image_hullIndices;
	std::vector<std::vector<cv::Point>> frame_hullIndices;

	std::vector<std::vector<int>> indices_triangles;

};

