#include <fstream>
#include <mutex>
#include <chrono>

#include "image_grabber_webots_direct.h"
#include "picture.pb.h"
static std::mutex lock_obj;
using namespace std::literals::chrono_literals;


ImageGrabberWebots::ImageGrabberWebots(int in_id, const int width, const int height) : ImageGrabber(in_id, width, height), max_message_len(700 * 480 * 4),
																					   queue_size(2), forced_remove(removeQueue()), msgq(boost::interprocess::open_or_create, "WEBOTS_PICTURE_COMMUNICATION", queue_size, max_message_len)
{
	if(forced_remove){
		std::cout << "remove success" << std::endl;
	}
	//msgq = boost::interprocess::message_queue(boost::interprocess::create_only, "WEBOTS_PICTURE_COMMUNICATION", queue_size, max_message_len);
}

ImageGrabberWebots::~ImageGrabberWebots()
{
	//queueの開放
	boost::interprocess::message_queue::remove("WEBOTS_PICTURE_COMMUNICATION");
}

int ImageGrabberWebots::grabFrame(void)
{
	//std::cerr << "----------------------now call grabframe------------------------------" << std::endl;
	webotsvision::CameraMeasurement picture_data;
	std::string first_received_data;
	std::string second_received_data;
	first_received_data.resize(max_message_len);
	second_received_data.resize(max_message_len);
	uint64_t recv_size = 0;
	unsigned int first_priority = 0;
	unsigned int second_priority = 0;

	msgq.receive(&first_received_data[0], first_received_data.size(), recv_size, first_priority);
	std::this_thread::sleep_for(3ms);
	msgq.receive(&second_received_data[0], second_received_data.size(), recv_size, second_priority);

	picture_data.ParseFromString(second_received_data);

	std::string recv_pic = picture_data.image();
	int64_t dbg = picture_data.simtime();
	//std::cerr << "----------------------simtime is " << dbg << "--------------------------" << std::endl;

	if (recv_pic.size() > 0)
	{
		static constexpr int webots_default_capture_width = 640;
		static constexpr int webots_default_capture_height = 480;
		/*if (recv_pic.size() != webots_default_capture_width * webots_default_capture_height * 4)
		{
			return -1;
		}*/
		cv::Mat raw_img = cv::Mat::ones(webots_default_capture_height, webots_default_capture_width, CV_8UC4);
		unsigned char *buf = raw_img.data;
		for (auto i = 0; i < recv_pic.size(); i++)
		{
			buf[i] = recv_pic[i];
		}
		cv::cvtColor(raw_img,raw_img,cv::COLOR_BGRA2BGR);
		//cv::imshow("image",raw_img);
		//std::this_thread::sleep_for(5000ms);
		//cv::waitKey(0);
		std::lock_guard<std::mutex> lock_img(lock_obj);
		cv::resize(raw_img, img, img.size());
	}
	else
	{
		return -1;
	}
	return 0;
}

int ImageGrabberWebots::getImage(cv::Mat &dst_img)
{
	std::lock_guard<std::mutex> lock_img(lock_obj);
	if (img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows)
	{
		std::cerr << "invalid image in image_grabber from webots" << std::endl;
		throw 0;
	}
	else
	{
		cv::cvtColor(img, dst_img, cv::COLOR_BGR2YCrCb);
	}
	return 0;
}

bool ImageGrabberWebots::removeQueue()
{
	return boost::interprocess::message_queue::remove("WEBOTS_PICTURE_COMMUNICATION");
}
