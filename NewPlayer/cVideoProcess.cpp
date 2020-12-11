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

static void draw_delaunay(cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color)
{
	std::vector<cv::Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	std::vector<cv::Point> pt(3);
	cv::Size size = img.size();
	cv::Rect rect(0, 0, size.width, size.height);

	for (int i = 0; i < triangleList.size(); i++) {
		cv::Vec6f t = triangleList[i];
		pt[0] = cv::Point(std::round(t[0]), std::round(t[1]));
		pt[1] = cv::Point(std::round(t[2]), std::round(t[3]));
		pt[2] = cv::Point(std::round(t[4]), std::round(t[5]));

		// Draw rectangles completely inside the image
		if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
			cv::line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
			cv::line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
			cv::line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
		}
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

	// Delaunay Triangulation
	std::vector<cv::Point2f> points;
	for (unsigned long j = 0; j < contours.size(); ++j) {
		for (int i = 0; i < hullIndices[j].size(); i++) {
			points.push_back(hullIndices[j][i]);
		}
	}

	cv::Rect rect(0, 0, dst_frame.cols, dst_frame.rows);
	cv::Subdiv2D subdiv(rect);

	for (int i = 0; i < points.size(); i++) {
		subdiv.insert(points[i]);
	}

	draw_delaunay(dst_frame, subdiv, (255, 255, 255));
}

void cVideoProcess::ProcessVideo()
{
	std::string file_path = ORIG_VIDEO_PATH;
	std::string file_path_new = NEW_VIDEO_PATH;
	std::string swap_file_path = SWAP_FILE_PATH;

	cv::VideoCapture cap;
	cv::Mat src_frame;
	cv::Mat dst_frame;

	// Open the image file
	cv::Mat src_swap_img = cv::imread(swap_file_path);
	cv::Mat dst_swap_img;

	ProcessFrame(src_swap_img, dst_swap_img);

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