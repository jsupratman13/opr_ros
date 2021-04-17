#include <random>
#include <chrono>
#include <algorithm>
#include <mutex>

#include "object_detector.h"
#include "../hpl_types.h"

std::mutex label_img_mutex_lock;
std::mutex label_img_for_visualize_mutex_lock;

ObjectDetector::ObjectDetector(const int w, const int h) : width(w), height(h)
{
}

ObjectDetector::~ObjectDetector()
{
}

void ObjectDetector::getColorResultImageData(cv::Mat &result)
{
	std::lock_guard<std::mutex> lock(label_img_for_visualize_mutex_lock);
	if(label_img_for_visualize.empty()) {
		result = cv::Mat::zeros(height, width, CV_16UC1);
	} else {
		result = label_img_for_visualize.clone();
	}
}

void ObjectDetector::setColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
}

void ObjectDetector::clearColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
}

void ObjectDetector::resetColorTable(unsigned short object_type)
{
}

void ObjectDetector::loadColorTable(std::string filename)
{
}

void ObjectDetector::saveColorTable(std::string filename)
{
}

void ObjectDetector::setImage(cv::Mat in_img)
{
	img = in_img;
}

void ObjectDetector::setImageSize(const int w, const int h)
{
	width = w;
	height = h;
}

std::vector<Particle> ObjectDetector::getBallParticle(void)
{
	// return empty particle vector
	std::vector<Particle> ret;
	return ret;
}

std::vector<struct BoundingBox> ObjectDetector::getBallBoundingBox(void)
{
	// return empty vector
	std::vector<struct BoundingBox> ret;
	return ret;
}

std::vector<struct BoundingBox> ObjectDetector::getGoalBoundingBox(void)
{
	// return empty vector
	std::vector<struct BoundingBox> ret;
	return ret;
}

std::vector<struct BoundingBox> ObjectDetector::getAlliedRobotBoundingBox(void)
{
	// return empty vector
	std::vector<struct BoundingBox> ret;
	return ret;
}

std::vector<struct BoundingBox> ObjectDetector::getEnemyRobotBoundingBox(void)
{
	// return empty vector
	std::vector<struct BoundingBox> ret;
	return ret;
}

bool ObjectDetector::setupYOLO(std::string config_file, std::string weight_file, std::vector<int> thresholds)
{
	return false;
}

