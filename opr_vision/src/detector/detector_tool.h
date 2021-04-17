#ifndef DETECTOR_TOOL_H
#define DETECTOR_TOOL_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../object_types.h"

namespace DetectorTool {
	void eraseWhiteLineOutsideField(cv::Mat &, const int=5);
	void eraseWhiteLineOutsideField(cv::Mat &, std::vector<int>, const int=5);
	void detectWhiteLine(cv::Mat &, std::vector<object_pos> &);
	void detectFieldEdge(cv::Mat &, std::vector<int> &, std::vector<int> &);
	void giftWrapping(std::vector<int> &, std::vector<int> &);
	void detectObstacle(std::vector<int> &, std::vector<int> &, std::vector<object_pos> &);
	void showFieldEdge(cv::Mat &, std::vector<int> &);
};

#endif // DETECTOR_TOOL_H

