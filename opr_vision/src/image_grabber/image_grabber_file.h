#ifndef FILE_GRABBER_H
#define FILE_GRABBER_H

#include <string>
#include <vector>
#include "image_grabber.h"

class ImageGrabberFile : public ImageGrabber
{
public:
	ImageGrabberFile(int id, const int w, const int h);
	~ImageGrabberFile() {}
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
private:
	std::size_t image_index;
	std::vector<std::string> image_file_list;
};

#endif // FILE_GRABBER_H

