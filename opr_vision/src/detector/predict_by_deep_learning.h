#ifndef PREDICT_BY_DEEP_LEARNING_H
#define PREDICT_BY_DEEP_LEARNING_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class DeepLearning
{
public:
	DeepLearning();
	~DeepLearning();
	cv::Mat predict(const cv::Mat &);
	std::size_t getInputImageSize(void);
	std::size_t getPredictedImageSize(void);
	void setImageSize(const cv::Mat &);
	void setImageSize(const std::size_t, const std::size_t);
private:
	const int tcp_port;
	std::size_t image_width;
	std::size_t image_height;
	std::size_t image_channels;
	const std::string address;
	std::vector<unsigned short> color_table_index;
	void createColorTableIndex(void);
};

#endif // PREDICT_BY_DEEP_LEARNING_H

