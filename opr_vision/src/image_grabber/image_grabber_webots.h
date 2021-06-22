#ifndef VREP_GRABBER_H
#define VREP_GRABBER_H

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "image_grabber.h"
#include <WebotsClient.h>

class ImageGrabberWebots : public ImageGrabber
{
public:
	ImageGrabberWebots(int id, const int w, const int h);
	~ImageGrabberWebots() {}
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
private:
	double prevtime;
};

#endif // VREP_GRABBER_H

