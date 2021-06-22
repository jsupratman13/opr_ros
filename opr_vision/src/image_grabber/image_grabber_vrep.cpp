#include <fstream>
#include <mutex>
#include <chrono>

#include "image_grabber_vrep.h"

static std::mutex lock_obj;

ImageGrabberVrep::ImageGrabberVrep(int in_id, const int width, const int height) : ImageGrabber(in_id, width, height)
{
	prevtime = static_cast<double>(client.getSimulationTime());
}

int ImageGrabberVrep::grabFrame(void)
{
	double curtime = static_cast<double>(client.getSimulationTime());
	constexpr double capture_interval = 0.1; // 100 msec
	while(curtime - prevtime < capture_interval) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		curtime = static_cast<double>(client.getSimulationTime());
		if(curtime < prevtime) {
			break;
		}
	}
	prevtime = curtime;
	hc::Image sim_img = client.getImage(id);
	if(sim_img.size() > 0) {
		constexpr int vrep_default_capture_width = 320;
		constexpr int vrep_default_capture_height = 240;
		if(sim_img.size() != vrep_default_capture_width * vrep_default_capture_height * 3) {
			return -1;
		}
		cv::Mat raw_img = cv::Mat::ones(vrep_default_capture_height, vrep_default_capture_width, CV_8UC3);
		unsigned char *buf = raw_img.data;
		for(auto i = 0; i < sim_img.size(); i++) {
			buf[i] = sim_img[i];
		}
		std::lock_guard<std::mutex> lock_img(lock_obj);
		cv::resize(raw_img, img, img.size());
	} else {
		return -1;
	}
	return 0;
}

int ImageGrabberVrep::getImage(cv::Mat &dst_img)
{
	std::lock_guard<std::mutex> lock_img(lock_obj);
	if(img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows) {
		std::cerr << "invalid image in image_grabber from vrep" << std::endl;
		throw 0;
	} else {
		cv::cvtColor(img, dst_img, CV_RGB2YCrCb);
	}
	return 0;
}

