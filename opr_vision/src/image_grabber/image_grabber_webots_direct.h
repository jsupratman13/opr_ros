#ifndef VREP_GRABBER_H
#define VREP_GRABBER_H

#include "image_grabber.h"
#include <boost/interprocess/ipc/message_queue.hpp>
//#include <SimulatorIPC.h>

class ImageGrabberWebots : public ImageGrabber
{
public:
	ImageGrabberWebots(int id, const int w, const int h);
	~ImageGrabberWebots();
	int getImage(cv::Mat &) override;
	int grabFrame(void) override;
private:
	double prevtime;
	bool forced_remove;//確実にmessage_queueをremoveする為にあるので宣言の順番を変えてはいけない。
    uint32_t queue_size;
    const int32_t max_message_len;
	bool removeQueue();
	boost::interprocess::message_queue msgq;


};

#endif // VREP_GRABBER_H

