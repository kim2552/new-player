#include "cVideoProcess.h"

cVideoProcess::cVideoProcess()
{
	ff_detector = dlib::get_frontal_face_detector();
	dlib::deserialize(SHAPE_PREDICTOR) >> shape_pred;
}

cVideoProcess::~cVideoProcess()
{

}

/** This function converts dlib::point to cv::Point and stores in a vector of landmarks
	This function is needed because in this implementation I have used opencv and dlib bothand they
	both have their own image processing library so when using a dlib method, its arguments should be
	as expected */
void dlib_point2cv_Point(dlib::full_object_detection& S, std::vector<cv::Point>& L, double& scale)
{
	for (unsigned int i = 0; i < S.num_parts(); ++i)
	{
		L.push_back(cv::Point(S.part(i).x() * (1 / scale), S.part(i).y() * (1 / scale)));
	}
}

void cVideoProcess::ProcessFrame(cv::Mat& src_frame, cv::Mat& dst_frame)
{
	// Frontal face detector
	dlib::array2d<dlib::rgb_pixel> dlib_frame;
	dlib::assign_image(dlib_frame, dlib::cv_image<dlib::rgb_pixel>(src_frame));
	std::vector<dlib::rectangle> dets = ff_detector(dlib_frame);
	
	std::vector<dlib::full_object_detection> shapes;
	double scale = 1;

	std::vector<std::vector<cv::Point>> contours;
	for (unsigned long j = 0; j < dets.size(); ++j) {
		std::vector<cv::Point> contour;

		dlib::full_object_detection shape = this->shape_pred(dlib_frame, dets[j]);
		shapes.push_back(shape);

		dlib_point2cv_Point(shape, contour, scale);
		contours.push_back(contour);
	}
	dst_frame = dlib::toMat(dlib_frame);

#if DEVELOP_MODE
	this->win.clear_overlay();
	this->win.set_image(dlib_frame);
	this->win.add_overlay(dlib::render_face_detections(shapes));
#endif

	// convex Hull
	std::vector<std::vector<cv::Point>> hullIndices(contours.size());
	for (unsigned long j = 0; j < contours.size(); ++j) {
		cv::convexHull(contours[j], hullIndices[j], false, false);
	}

	for (unsigned long j = 0; j < contours.size(); ++j) {
		cv::drawContours(dst_frame, hullIndices, (int)j, (0, 255, 0), 2);
	}
}

void cVideoProcess::ProcessVideo()
{
	std::string file_path = ORIG_VIDEO_PATH;
	std::string file_path_new = NEW_VIDEO_PATH;

	cv::VideoCapture cap;
	cv::Mat src_frame;
	cv::Mat dst_frame;

	// Open the video file
	cap.open(file_path);
	if (!cap.isOpened()) {
		wxLogDebug("Error opening video stream or file");
	}

	// Need first frame to get frame size
	cap >> src_frame;

	float fps = cap.get(cv::CAP_PROP_FPS);																// Get FPS for new video
	int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');											// Codec for mp4
	cv::VideoWriter new_video(file_path_new, fourcc, fps, cv::Size(src_frame.cols, src_frame.rows), 1);	// Video writer

	// Loop through the rest of the frames
	while(1){
		if (src_frame.empty()) {
			break;
		} else {

			ProcessFrame(src_frame, dst_frame);

			new_video.write(dst_frame);
		}

		cap >> src_frame;
	}

	// Release video capture and writer
	cap.release();
	new_video.release();
}