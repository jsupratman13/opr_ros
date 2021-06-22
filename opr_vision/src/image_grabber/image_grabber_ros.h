#ifndef ROS_GRABBER_H
#define ROS_GRABBER_H

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "image_grabber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

class ImageGrabberROS : public ImageGrabber
{
public:
    ImageGrabberROS(int id, const int w, const int h);
    int getImage(cv::Mat &) override;
    int grabFrame(void) override;
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
private:
    cv_bridge::CvImagePtr cv_ptr;
    // ros::NodeHandle nh_;
    // image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

#endif // ROS_GRABBER_H
