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
static void dlib_point2cv_Point(dlib::full_object_detection& dlib_points, std::vector<cv::Point>& cv_points, double& scale)
{
	for (unsigned int i = 0; i < dlib_points.num_parts(); ++i)
	{
		cv_points.push_back(cv::Point(dlib_points.part(i).x() * (1 / scale), dlib_points.part(i).y() * (1 / scale)));
	}
}

static void DrawDelaunay(cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color)
{
	std::vector<cv::Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	std::vector<cv::Point> pt(3);
	cv::Size size = img.size();
	cv::Rect rect(0, 0, size.width, size.height);

	for (unsigned int i = 0; i < triangleList.size(); i++) {
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

void cVideoProcess::ProcessFrame(cv::Mat& src_frame, cv::Mat& dst_frame, std::vector<cv::VideoWriter> videos)
{
	/** Frontal face detector **/
	dlib::array2d<dlib::rgb_pixel> dlib_frame;
	dlib::assign_image(dlib_frame, dlib::cv_image<dlib::rgb_pixel>(src_frame));
	std::vector<dlib::rectangle> dets = ff_detector(dlib_frame);
	
	std::vector<dlib::full_object_detection> shapes;
	double scale = 1;

	std::vector<std::vector<cv::Point>> contours;
	for (unsigned int j = 0; j < dets.size(); j++) {
		std::vector<cv::Point> contour;

		dlib::full_object_detection shape = this->shape_pred(dlib_frame, dets[j]);
		shapes.push_back(shape);

		dlib_point2cv_Point(shape, contour, scale);
		contours.push_back(contour);
	}
	/*****************************************/

	/** Display Facial Landmarks **/
	cv::Mat fmat = src_frame.clone();
	for (unsigned int j = 0; j < contours.size(); j++) {
		for(unsigned int i = 0; i < contours[j].size(); i++)
		cv::circle(fmat, contours[j][i], 1, (0, 255, 255), 1);
	}
	if (videos.size() > 0)
		videos[0].write(fmat);	//Landmark Video Write
	fmat.release();
	/*****************************************/

	/** Convex Hull **/
	std::vector<std::vector<cv::Point>> hullIndices(contours.size());
	for (unsigned int j = 0; j < contours.size(); j++) {
		cv::convexHull(contours[j], hullIndices[j], false, false);
	}
	/*****************************************/

	/** Display Convex Hull **/
	cv::Mat chmat = src_frame.clone();
	for (unsigned int j = 0; j < contours.size(); j++) {
		cv::drawContours(chmat, hullIndices, j, (0, 255, 0), 2);
	}
	if (videos.size() > 1)
		videos[1].write(chmat);	//Convex Hull Video Write
	chmat.release();
	/*****************************************/

	/** Delaunay Triangulation **/
	cv::Mat dmat = src_frame.clone();
	std::vector<cv::Point2f> points;
	for (unsigned int j = 0; j < contours.size(); j++) {
		for (int i = 0; i < hullIndices[j].size(); i++) {
			points.push_back(hullIndices[j][i]);
		}
	}

	cv::Rect rect(0, 0, dmat.cols, dmat.rows);
	cv::Subdiv2D subdiv(rect);

	for (unsigned int i = 0; i < points.size(); i++) {
		subdiv.insert(points[i]);
	}

	DrawDelaunay(dmat, subdiv, (255, 255, 255));
	if (videos.size() > 2)
		videos[2].write(dmat);	//Convex Hull Video Write
	dmat.release();
	/*****************************************/
}

void cVideoProcess::ProcessVideo()
{
	std::string file_path = ORIG_VIDEO_PATH;
	std::string file_path_new = NEW_VIDEO_PATH;
	std::string swap_file_path = SWAP_FILE_PATH;
	std::string landmark_path = LANDMARK_VIDEO_PATH;
	std::string convex_path = CONVEX_VIDEO_PATH;
	std::string delaunay_path = DELAUNAY_VIDEO_PATH;

	cv::VideoCapture cap;
	cv::Mat src_frame;
	cv::Mat dst_frame;

	// Open the image file
	cv::Mat src_swap_img = cv::imread(swap_file_path);
	cv::Mat dst_swap_img;

	std::vector<cv::VideoWriter> videos;

	ProcessFrame(src_swap_img, dst_swap_img, videos);

	// Open the video file
	cap.open(file_path);
	if (!cap.isOpened()) {
		wxLogDebug("Error opening video stream or file");
	}

	// Need first frame to get frame size
	cap >> src_frame;

	float fps = cap.get(cv::CAP_PROP_FPS);																// Get FPS for new video
	int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');											// Codec for mp4
	cv::VideoWriter new_video(file_path_new, fourcc, fps, cv::Size(src_frame.cols, src_frame.rows), 1);
	cv::VideoWriter landmark_video(landmark_path, fourcc, fps, cv::Size(src_frame.cols, src_frame.rows), 1);
	cv::VideoWriter convex_video(convex_path, fourcc, fps, cv::Size(src_frame.cols, src_frame.rows), 1);	
	cv::VideoWriter delaunay_video(delaunay_path, fourcc, fps, cv::Size(src_frame.cols, src_frame.rows), 1);
	videos.push_back(landmark_video);
	videos.push_back(convex_video);
	videos.push_back(delaunay_video);

	// Loop through the rest of the frames
	while(1){
		if (src_frame.empty()) {
			break;
		} else {

			ProcessFrame(src_frame, dst_frame, videos);

			new_video.write(dst_frame);
		}

		cap >> src_frame;
	}

	// Release video capture and writer
	cap.release();
	for (unsigned int i = 0; i < videos.size(); i++) {
		videos[i].release();
	}
	new_video.release();
}