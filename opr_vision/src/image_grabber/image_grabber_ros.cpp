#include <fstream>
#include <mutex>
#include "image_grabber_ros.h"

static std::mutex lock_obj;

ImageGrabberROS::ImageGrabberROS(int in_id, const int width, const int height) : ImageGrabber(in_id, width, height)
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("/camera_sensor/image_raw", 1, &ImageGrabberROS::image_callback, this);
}

int ImageGrabberROS::grabFrame()
{
}

void ImageGrabberROS::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    // non direct version
    // cv::Mat raw_img = cv::Mat::ones(cv_ptr->image.cols, cv_ptr->image.rows, CV_8UC3);
    cv::Mat raw_img = cv::Mat::ones(height, width, CV_8UC3);
    unsigned char *destination = raw_img.data;
    unsigned char *source = (unsigned char *)cv_ptr->image.data;
    // int size = cv_ptr->image.rows * cv_ptr->image.cols * 3;
    int size = width * height * 3;
    for(int i = 0; i < size; i++){
        *destination ++ = *source ++;
    }
    std::lock_guard<std::mutex> lock_img(lock_obj);
    cv::resize(raw_img, img, img.size());
    
    return ;
}

int ImageGrabberROS::getImage(cv::Mat &dst_img)
{
    std::lock_guard<std::mutex> lock_img(lock_obj);
    if (img.empty() || img.cols != dst_img.cols || img.rows != dst_img.rows)
    {
        std::cerr << "invalid image in image_grabber from ros topic" << std::endl;
        throw 0;
    }
    else
    {
        cv::cvtColor(img, dst_img, cv::COLOR_BGR2YCrCb);
    }
    return 0;
}
