#include <boost/asio.hpp>

namespace asio = boost::asio;
using asio::ip::tcp;

#include "predict_by_deep_learning.h"
#include "hpl_types.h"

DeepLearning::DeepLearning() : tcp_port(19000), image_width(320), image_height(240), image_channels(3), address("127.0.0.1"), color_table_index(256, 0)
{
	createColorTableIndex();
}

DeepLearning::~DeepLearning()
{
}

size_t DeepLearning::getInputImageSize(void)
{
	return image_width * image_height * image_channels;
}

size_t DeepLearning::getPredictedImageSize(void)
{
	return image_width * image_height * sizeof(unsigned char);
}

cv::Mat DeepLearning::predict(const cv::Mat &input_img)
{
	asio::io_service io_service;
	tcp::socket socket(io_service);

	boost::system::error_code error;
	socket.connect(tcp::endpoint(asio::ip::address::from_string(address), tcp_port), error);

	if(error) {
		std::cerr << "connect faild: " << error.message() << std::endl;
		throw 0;
	}

	auto send_data = asio::buffer(input_img.data, getInputImageSize());
	asio::write(socket, send_data, error);
	socket.close();
	if(error) {
		std::cerr << "send faild: " << error.message() << std::endl;
		throw 0;
	}

	socket.connect(tcp::endpoint(asio::ip::address::from_string(address), tcp_port), error);
	asio::streambuf receive_buffer;
	asio::read(socket, receive_buffer, asio::transfer_at_least(getPredictedImageSize()), error);
	socket.close();
	if(error) {
		std::cerr << "receive faild: " << error.message() << std::endl;
		throw 0;
	}
	const size_t receive_size = receive_buffer.size();
	if(receive_size != getPredictedImageSize()) {
		std::cerr << "no match image size" << std::endl;
		std::cerr << "received size: " << receive_size << std::endl;
		throw 0;
	}
	const unsigned char *predicted_image_data = asio::buffer_cast<const unsigned char *>(receive_buffer.data());
	cv::Mat predicted_image(image_height, image_width, CV_8UC1);
	for(std::size_t i = 0; i < getPredictedImageSize() / sizeof(unsigned char); i++) {
		//predicted_image.data[i] = color_table_index[predicted_image_data[i]];
		predicted_image.data[i] = predicted_image_data[i];
	}

	return predicted_image;
}

void DeepLearning::setImageSize(const cv::Mat &image)
{
	image_width = image.cols;
	image_height = image.rows;
}

void DeepLearning::setImageSize(const std::size_t width, const std::size_t height)
{
	image_width = width;
	image_height = height;
}

void DeepLearning::createColorTableIndex(void)
{
	const int background = 0;
	const int white = 1;
	const int ball = 2;
	const int green = 3;
	const int goal_post = 4;
	color_table_index[background] = 0;
	color_table_index[white]      = 1 << COLOR_WHITE;
	color_table_index[ball]       = 1 << COLOR_BALL;
	color_table_index[green]      = 1 << COLOR_GREEN;
	//color_table_index[goal_post]  = 1 << COLOR_WHITE;
}

