#include <fstream>
#include <mutex>
#include <chrono>
#include "image_grabber_webots.h"

static std::mutex lock_obj;
using namespace boost::interprocess;

ImageGrabberWebots::ImageGrabberWebots(int in_id, const int width, const int height) : ImageGrabber(in_id, width, height)
{
	managed_shared_memory m_shm(open_or_create, "WebotSharedMemory", sizeof(WebotsClient)+10000);
	std::pair<interprocess_mutex *, std::size_t> mx = m_shm.find<interprocess_mutex>("WebotSharedMemoryMutex");
	if(!mx.first) exit(1);
	interprocess_mutex *mutex = mx.first;
	std::pair<WebotsClient *, std::size_t> clientPair = m_shm.find<WebotsClient>("WebotsClient");
	if(!clientPair.first) exit(1);
	WebotsClient *client = clientPair.first;
	scoped_lock<interprocess_mutex> *lock = new scoped_lock<interprocess_mutex>(*mutex);	
	prevtime = static_cast<double>(client->time/1000.0);
	delete lock;
}

int ImageGrabberWebots::grabFrame(void)
{
	managed_shared_memory m_shm(open_or_create, "WebotSharedMemory", sizeof(WebotsClient)+10000);
	std::pair<interprocess_mutex *, std::size_t> mx = m_shm.find<interprocess_mutex>("WebotSharedMemoryMutex");
	if(!mx.first) exit(1);
	interprocess_mutex *mutex = mx.first;
	std::pair<WebotsClient *, std::size_t> clientPair = m_shm.find<WebotsClient>("WebotsClient");
	if(!clientPair.first) exit(1);
	WebotsClient *client = clientPair.first;

	double curtime = static_cast<double>(client->time/1000.0);
	constexpr double capture_interval = 0.1; // 100 msec
	while(curtime - prevtime < capture_interval) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		curtime = static_cast<double>(client->time/1000.0);
		if(curtime < prevtime) {
			break;
		}
	}
	prevtime = curtime;
	cv::Mat raw_img = cv::Mat::ones(client->camera.height, client->camera.width, CV_8UC3);
	unsigned char *distination = raw_img.data;
	unsigned char *source = (unsigned char *)client->camera.image;
	int size = client->camera.width * client->camera.height * 3;
	for(int i = 0; i < size; i++) {
		*distination ++ = *source ++;
	}
	std::lock_guard<std::mutex> lock_img(lock_obj);
	cv::resize(raw_img, img, img.size());
	
	return 0;
}

int ImageGrabberWebots::getImage(cv::Mat &dst_img)
{
	std::lock_guard<std::mutex> lock_img(lock_obj);
	if(img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows) {
		std::cerr << "invalid image in image_grabber from vrep" << std::endl;
		throw 0;
	} else {
		cv::cvtColor(img, dst_img, cv::COLOR_RGB2YCrCb);
	}
	return 0;
}

