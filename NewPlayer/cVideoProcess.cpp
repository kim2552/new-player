#include "cVideoProcess.h"

cVideoProcess::cVideoProcess()
{

}

cVideoProcess::~cVideoProcess()
{

}

void cVideoProcess::ProcessFrame(cv::Mat& frame)
{
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	// Convert frame to grayscale
	cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

	// Apply Sobel filter
	cv::Sobel(frame, grad_x, 3, 1, 0);
	cv::Sobel(frame, grad_y, 3, 0, 1);

	cv::convertScaleAbs(grad_x, abs_grad_x);
	cv::convertScaleAbs(grad_y, abs_grad_y);

	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, frame);
}

void cVideoProcess::ProcessVideo()
{
	std::string file_path = ORIG_VIDEO_PATH;
	std::string file_path_new = NEW_VIDEO_PATH;

	cv::VideoCapture cap;
	cv::Mat frame;

	// Open the video file
	cap.open(file_path);
	if (!cap.isOpened()) {
		wxLogDebug("Error opening video stream or file");
	}

	// Need first frame to get frame size
	cap >> frame;

	float fps = cap.get(cv::CAP_PROP_FPS);														// Get FPS for new video
	int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');									// Codec for mp4
	cv::VideoWriter new_video(file_path_new, fourcc, fps, cv::Size(frame.cols, frame.rows), 0);	// Video writer

	// Loop through the rest of the frames
	while(1){
		if (frame.empty()) {
			break;
		} else {

			ProcessFrame(frame);

			new_video.write(frame);
		}

		cap >> frame;
	}

	// Release video capture and writer
	cap.release();
	new_video.release();
}