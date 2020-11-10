#include "cVideoProccess.h"

cVideoProccess::cVideoProccess()
{

}

cVideoProccess::~cVideoProccess()
{

}

void cVideoProccess::ProcessVideo()
{
	std::string file_path = "C:/Users/jooho/Downloads/test_video.mp4";
	std::string file_path_new = "C:/Users/jooho/Downloads/test_video_new.mp4";

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
	cv::VideoWriter new_video(file_path_new, fourcc, fps, cv::Size(frame.cols, frame.rows));	// Video writer

	// Loop through the rest of the frames
	while(1){
		if (frame.empty()) {
			break;
		} else {
			new_video.write(frame);
		}

		cap >> frame;
	}

	// Release video capture and writer
	cap.release();
	new_video.release();
}