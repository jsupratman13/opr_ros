#ifndef VREP_GRABBER_H
#define VREP_GRABBER_H

#include "image_grabber.h"

#include <SimulatorIPC.h>

class ImageGrabberVrep : public ImageGrabber
{
public:
	ImageGrabberVrep(int id, const int w, const int h);
	~ImageGrabberVrep() {}
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
private:
	SimulatorIPCClient client;
	double prevtime;
};

#endif // VREP_GRABBER_H

