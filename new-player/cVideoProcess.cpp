#include "cVideoProcess.h"

cVideoProcess::cVideoProcess()
{
	ff_detector = dlib::get_frontal_face_detector();
	dlib::deserialize(SHAPE_PREDICTOR) >> shape_pred;
	show_intermediate_steps = true;
}

cVideoProcess::~cVideoProcess()
{

}

void cVideoProcess::ToggleIntermediateSteps()
{
	show_intermediate_steps = !show_intermediate_steps;
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

void cVideoProcess::DelaunayTriangulation(cv::Mat& img, cv::Subdiv2D& subdiv, std::vector<cv::Point> points, cv::Scalar delaunay_color, bool update_tri_indices)
{
	cv::Rect rect(0, 0, img.cols, img.rows);
	subdiv.initDelaunay(rect);

	for (unsigned int i = 0; i < points.size(); i++) {
		subdiv.insert(points[i]);
	}

	std::vector<cv::Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	std::vector<cv::Point> pt(3);

	for (unsigned int i = 0; i < triangleList.size(); i++) {
		cv::Vec6f t = triangleList[i];
		pt[0] = cv::Point(std::round(t[0]), std::round(t[1]));
		pt[1] = cv::Point(std::round(t[2]), std::round(t[3]));
		pt[2] = cv::Point(std::round(t[4]), std::round(t[5]));

		if (update_tri_indices) {
			std::vector<int> triangle;

			bool check1, check2, check3;
			check1 = check2 = check3 = false;

			int index_pt1, index_pt2, index_pt3;

			for (unsigned int j = 0; j < points.size(); j++) {
				if (points[j] == pt[0]) {
					index_pt1 = j;
					check1 = true;
				}
				if (points[j] == pt[1]) {
					index_pt2 = j;
					check2 = true;
				}
				if (points[j] == pt[2]) {
					index_pt3 = j;
					check3 = true;
				}
			}

			if (check1 == true && check2 == true && check3 == true) {
				triangle = { index_pt1, index_pt2, index_pt3 };
				indices_triangles.push_back(triangle);
			}
		}

		// Draw rectangles completely inside the image
		if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
			cv::line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
			cv::line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
			cv::line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
		}
	}
}

//TODO::Optimize algorithm to reduce time
std::vector<std::vector<cv::Point>> cVideoProcess::FrontalFaceLandmarks(cv::Mat& src)
{
	dlib::array2d<dlib::rgb_pixel> dlib_frame;
	dlib::assign_image(dlib_frame, dlib::cv_image<dlib::rgb_pixel>(src));
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
	return contours;
}

std::vector<std::vector<cv::Point>> cVideoProcess::HullIndicies(std::vector<std::vector<cv::Point>> contours)
{
	std::vector<std::vector<cv::Point>> hull_indicies(contours.size());
	for (unsigned int j = 0; j < contours.size(); j++) {
		cv::convexHull(contours[j], hull_indicies[j], false, false);
	}
	return hull_indicies;
}

void cVideoProcess::ProcessSourceFrame(cv::Mat& src_frame, cv::Mat& dst_frame, cv::Subdiv2D& subdiv, std::vector<cv::VideoWriter> videos)
{
	std::vector<cv::Point> points;

	std::vector<std::vector<cv::Point>> contours;
	contours = FrontalFaceLandmarks(src_frame);
	for (unsigned int j = 0; j < contours.size(); j++) {
		for (unsigned int i = 0; i < contours[j].size(); i++) {
			points.push_back(contours[j][i]);						// points saved for delaunay triangulation
		}
	}

	frame_hullIndices = HullIndicies(contours);

	/** Delaunay Triangulation **/
	cv::Mat dmat = src_frame.clone();
	DelaunayTriangulation(dmat, subdiv, points, (255, 255, 255), true);
	if (videos.size() > 2)
		videos[2].write(dmat);	//Convex Hull Video Write
	dst_frame = dmat.clone();
	dmat.release();
	/*****************************************/

	image_landmark_points = points;
}

cv::Mat cVideoProcess::TriangleMask(cv::Rect rectangle, std::vector<cv::Point> triangle)
{
	cv::Mat triangle_cropped_mask = cv::Mat::zeros(cv::Size(rectangle.width, rectangle.height), CV_8U);
	std::vector<cv::Point> points;
	for (unsigned int j = 0; j < 3; j++) {
		points.push_back(cv::Point((int)(triangle[j].x - rectangle.x), (int)(triangle[j].y - rectangle.y)));
	}
	cv::fillConvexPoly(triangle_cropped_mask, points, cv::Scalar(255, 255, 255));
	return triangle_cropped_mask;
}

cv::Mat cVideoProcess::WarpTriangle(cv::Mat img_triangle, std::vector<cv::Point> src_triangle, std::vector<cv::Point> dst_triangle, cv::Rect src_rectangle, cv::Rect dst_rectangle, cv::Mat mask)
{
	cv::Mat warp_mat;
	cv::Mat img_warped_triangle;
	cv::Mat img_warped_triangle_cropped;
	std::vector<cv::Point2f> img_points2f, frame_points2f;
	for (unsigned int j = 0; j < 3; j++) {
		img_points2f.push_back(cv::Point2f((src_triangle[j].x - src_rectangle.x), (src_triangle[j].y - src_rectangle.y)));
		frame_points2f.push_back(cv::Point2f((dst_triangle[j].x - dst_rectangle.x), (dst_triangle[j].y - dst_rectangle.y)));
	}
	warp_mat = cv::getAffineTransform(img_points2f, frame_points2f);
	cv::warpAffine(img_triangle, img_warped_triangle, warp_mat, cv::Size(dst_rectangle.width, dst_rectangle.height));
	cv::bitwise_and(img_warped_triangle, img_warped_triangle, img_warped_triangle_cropped, mask);

	return img_warped_triangle_cropped;
}

void cVideoProcess::ReconstructFace(cv::Mat& img, cv::Rect boundingRect, cv::Mat warped_triangle)
{
	cv::Mat frame_new_face_rect_area, frame_new_face_rect_area_grey;

	img(boundingRect).copyTo(frame_new_face_rect_area);

	cv::cvtColor(frame_new_face_rect_area, frame_new_face_rect_area_grey, cv::COLOR_BGR2GRAY);

	cv::Mat triangles_designed_mask;
	cv::threshold(frame_new_face_rect_area_grey, triangles_designed_mask, 1, 255, cv::THRESH_BINARY_INV);

	cv::Mat frame_warped_triangle_cropped;
	cv::bitwise_and(warped_triangle, warped_triangle, frame_warped_triangle_cropped, triangles_designed_mask);

	cv::Mat frame_new_face_rect_area_with_triangle;
	cv::add(frame_new_face_rect_area, frame_warped_triangle_cropped, frame_new_face_rect_area_with_triangle);

	frame_new_face_rect_area_with_triangle.copyTo(img(boundingRect));
}

void cVideoProcess::ProcessTriangulation(cv::Mat& img, cv::Mat& frame, std::vector<std::vector<cv::Point>>& hullIndices)
{
	cv::Mat frame_new_face = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), frame.type());

	cv::Rect img_rect, frame_rect;
	std::vector<cv::Point> img_tri, frame_tri;

	for (unsigned int i = 0; i < indices_triangles.size(); i++) {
		img_tri.push_back(image_landmark_points[indices_triangles[i][0]]);
		img_tri.push_back(image_landmark_points[indices_triangles[i][1]]);
		img_tri.push_back(image_landmark_points[indices_triangles[i][2]]);

		frame_tri.push_back(frame_landmark_points[indices_triangles[i][0]]);
		frame_tri.push_back(frame_landmark_points[indices_triangles[i][1]]);
		frame_tri.push_back(frame_landmark_points[indices_triangles[i][2]]);

		img_rect = cv::boundingRect(img_tri);
		frame_rect = cv::boundingRect(frame_tri);

		cv::Mat img_cropped_triangle;
		img(img_rect).copyTo(img_cropped_triangle);

		// Create triangle mask for image
		cv::Mat img_triangle_cropped_mask = TriangleMask(img_rect, img_tri);

		// Create triangle mask for frame
		cv::Mat frame_triangle_cropped_mask = TriangleMask(frame_rect, frame_tri);

		// Warp the triangles
		cv::Mat img_warped_triangle_cropped = WarpTriangle(img_cropped_triangle, img_tri, frame_tri, img_rect, frame_rect, frame_triangle_cropped_mask);

		// Reconstruct the frame face
		ReconstructFace(frame_new_face, frame_rect, img_warped_triangle_cropped);

		img_tri.clear();
		frame_tri.clear();
	}

	cv::Mat frame_grey;
	cv::cvtColor(frame, frame_grey, cv::COLOR_BGR2GRAY);
	std::vector<cv::Point> convexHull = hullIndices[0];

	cv::Mat frame_face_mask, frame_head_mask;
	frame_head_mask = cv::Mat::zeros(cv::Size(frame_grey.cols, frame_grey.rows), frame_grey.type());
	cv::fillConvexPoly(frame_head_mask, convexHull, cv::Scalar(255, 255, 255));
	cv::bitwise_not(frame_head_mask, frame_face_mask);

	cv::Mat seamless_clone = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), frame.type());
	cv::Mat frame_head_noface;
	cv::bitwise_and(frame, frame, frame_head_noface, frame_face_mask);

	cv::Mat resulting_face;
	cv::add(frame_head_noface, frame_new_face, resulting_face);

	cv::Rect convexHull_rect = cv::boundingRect(convexHull);
	cv::Point center_face = cv::Point((int)((convexHull_rect.x + convexHull_rect.x + convexHull_rect.width) / 2), (int)((convexHull_rect.y + convexHull_rect.y + convexHull_rect.height) / 2));
	cv::seamlessClone(resulting_face, frame, frame_head_mask, center_face, seamless_clone, cv::NORMAL_CLONE);
	seamless_clone.copyTo(frame);
}

void cVideoProcess::ProcessDestinationFrame(cv::Mat& src_frame, cv::Mat& dst_frame, cv::Subdiv2D& subdiv, std::vector<cv::VideoWriter> videos)
{
	std::vector<cv::Point> points;

	std::vector<std::vector<cv::Point>> contours;
	contours = FrontalFaceLandmarks(src_frame);

	/** Display Facial Landmarks **/
	cv::Mat fmat = src_frame.clone();
	for (unsigned int j = 0; j < contours.size(); j++) {
		for (unsigned int i = 0; i < contours[j].size(); i++) {
			points.push_back(contours[j][i]);						// points saved for delaunay triangulation
			cv::circle(fmat, contours[j][i], 1, (0, 255, 255), 1);
		}
	}
	if (show_intermediate_steps) {
		if (videos.size() > 0)
			videos[0].write(fmat);	//Landmark Video Write
	}
	fmat.release();
	/*****************************************/

	frame_hullIndices = HullIndicies(contours);

	/** Display Convex Hull **/
	cv::Mat chmat = src_frame.clone();
	for (unsigned int j = 0; j < contours.size(); j++) {
		cv::drawContours(chmat, frame_hullIndices, j, (0, 255, 0), 2);
	}
	if (show_intermediate_steps) {
		if (videos.size() > 1)
			videos[1].write(chmat);	//Convex Hull Video Write
	}
	chmat.release();
	/*****************************************/

	/** Delaunay Triangulation **/
	cv::Mat dmat = src_frame.clone();
	DelaunayTriangulation(dmat, subdiv, points, (255, 255, 255), false);
	if (show_intermediate_steps) {
		if (videos.size() > 2)
			videos[2].write(dmat);	//Convex Hull Video Write
	}
	dst_frame = dmat.clone();
	dmat.release();
	/*****************************************/

	frame_landmark_points = points;
}

void cVideoProcess::ProcessVideo(std::string original_video_file_path, std::string image_file_path)
{
	std::string file_path = original_video_file_path;
	std::string file_path_new = NEW_VIDEO_PATH;

	std::string swap_file_path = image_file_path;
	std::string swap_file_path_new = NEW_SWAP_FILE_PATH;

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

	ProcessSourceFrame(src_swap_img, dst_swap_img, image_subdiv, videos);
	cv::imwrite(swap_file_path_new, dst_swap_img);

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

			ProcessDestinationFrame(src_frame, dst_frame, frame_subdiv, videos);
			if (!frame_landmark_points.empty()) {
				ProcessTriangulation(src_swap_img, src_frame, frame_hullIndices);
			}

			new_video.write(src_frame);

			frame_landmark_points.clear();
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