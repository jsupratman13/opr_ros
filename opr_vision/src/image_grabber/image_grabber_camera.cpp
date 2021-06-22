#include <iostream>
#include <mutex>

#include "image_grabber_camera.h"

static std::mutex lock_obj;

ImageGrabberCamera::ImageGrabberCamera(int in_id, const int w, const int h) : ImageGrabber(in_id, w, h), camera_device_number(0), capture(camera_device_number), bauto(false), shutter(0), gain(0), white_balance(0)
{
	if(!capture.isOpened()) {
		std::cerr << "Camera open error" << std::endl;;
		throw 0;
	}
	capture.set(cv::CAP_PROP_FRAME_WIDTH, (double)width);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, (double)height);
}

int ImageGrabberCamera::grabFrame(void)
{
	std::lock_guard<std::mutex> lock_img(lock_obj);
	capture >> img;
	return 0;
}

int ImageGrabberCamera::getImage(cv::Mat &dst_img)
{
	if(img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows) {
		std::cerr << "invalid image in image_grabber from vrep" << std::endl;;
		throw 0;
	} else {
		std::lock_guard<std::mutex> lock_img(lock_obj);
		cv::cvtColor(img, dst_img, cv::COLOR_BGR2YCrCb);
	}
	return 0;
}

void ImageGrabberCamera::setCameraSettings(bool in_bauto, int in_shutter, int in_gain, int in_white_balance)
{
	bauto = in_bauto;
	shutter = in_shutter;
	gain = gain;
	white_balance = in_white_balance;

	capture.set(cv::CAP_PROP_GAIN, (double)gain);
}

