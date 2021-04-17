#ifndef VISION_H
#define VISION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include "opr_msgs/Color.h"
#include "opr_msgs/SetColor.h"

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QImage>

#include "hpl_types.h"
#include "detector/object_detector.h"

class Vision
{
public:
    Vision();
    ~Vision();
    void setLensType(const int);
    std::vector<Pos2D> getDetectedObject(Object_T, Object_Ownership_T = OWNERSHIP_ANY);
    int getWhiteLinePos(std::vector<NPos2D> &);
    void getImageInfo(int &, int &, int &);
    void getImageInfo(const IplImage *, int &, int &, int &);
    void getImageData(char *);
    void getColorResultData(char *);
    void setColorTable(const opr_msgs::SetColor::ConstPtr &msg);
    bool loadColorTable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool saveColorTable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool clearColorTable(opr_msgs::Color::Request &req, opr_msgs::Color::Response &res);
    void getCameraSettings(bool &, int &, int &, int &);
    void setCameraSettings(bool, int, int, int);
    void readCameraSettingFile(void);
    void writeCameraSettingFile(void);
    void setResolution(const int);
    int getBallParticle(std::vector<struct ball_particle_T> &);
    std::vector<struct BoundingBox> getBallBoundingBox(void);
    std::vector<struct BoundingBox> getGoalBoundingBox(void);
    std::vector<struct BoundingBox> getAlliedRobotBoundingBox(void);
    std::vector<struct BoundingBox> getEnemyRobotBoundingBox(void);
    void getUpdatedImage(IplImage **);
    int getWhiteLinePosFromImage(std::vector<NPos2D> &, const IplImage *);
    bool setUseYOLO(bool, std::string, std::string, std::vector<int>);
    std::string detectQRcode(void);
    IplImage *ipl;
    void update();
private:
    std::string color_table_filename;
    std::vector<object_pos> objects;
    std::vector<object_pos> white_line;
    cv::Mat img;
    cv::Mat view_img;
    std::unique_ptr<ObjectDetector> object_detector;
    bool terminated;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
protected:
    ros::NodeHandle nh_;
    ros::ServiceServer save_server_;
    ros::ServiceServer load_server_;
    ros::ServiceServer clear_server_;
    ros::Subscriber set_color_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    //cv_bridge::CvImagePtr cv_ptr;
};

#endif // VISION_H

