#include <iostream>
#include <fstream>
#include <mutex>

#include "image_grabber_file.h"

static std::mutex lock_obj;

ImageGrabberFile::ImageGrabberFile(int in_id, const int w, const int h) : ImageGrabber(in_id, w, h), image_index(0)
{
	std::string image_list_filename("image_list.txt");
	std::ifstream ifs(image_list_filename);
	if(!ifs) {
		std::cerr << "No image list file [in vision/image_grabber_file]" << std::endl;
		throw 0;
	}
	std::string line;
	while(std::getline(ifs, line)) {
		image_file_list.push_back(line);
	}
}

int ImageGrabberFile::grabFrame(void)
{
	return 0;
}

int ImageGrabberFile::getImage(cv::Mat &dst_img)
{
	img = cv::imread(image_file_list[image_index]);
	image_index++;
	if(image_index >= image_file_list.size())
		image_index = 0;
	if(img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows) {
		cv::resize(img, img, cv::Size(dst_img.cols, dst_img.rows));
	}
	std::unique_lock<std::mutex> lock_img(lock_obj);
	cv::cvtColor(img, dst_img, cv::COLOR_BGR2YCrCb);
	return 0;
}

