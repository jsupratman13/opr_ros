#ifndef CAMERA_GRABBER_H
#define CAMERA_GRABBER_H

#include "image_grabber.h"

class ImageGrabberCamera : public ImageGrabber
{
public:
	ImageGrabberCamera(int id, const int w, const int h);
	~ImageGrabberCamera() {}
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
	void setCameraSettings(bool bauto, int shutter, int gain, int white_balance) override;
private:
	int camera_device_number;
	cv::VideoCapture capture;
	bool bauto;
	int shutter;
	int gain;
	int white_balance;
};

#endif // CAMERA_GRABBER_H

