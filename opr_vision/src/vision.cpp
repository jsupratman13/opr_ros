#include <cstdlib>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
//#include <experimental/filesystem>
#include <iostream>

#include <zbar.h>

#include "vision.h"
#include "detector/detector_color_table.h"
//#include "detector/detector_wl.h"

static const std::string NAME = "test_wind";
static const std::string NAME2 = "test2_win";
static const std::string NAME3 = "test3_win";
// static std::mutex mutex_lock;
// static std::mutex mutex_objects;
// static std::mutex mutex_whiteline;

cv::Mat QImageToMat(QImage image)
{
    cv::Mat mat;
    switch(image.format()){
        case QImage::Format_ARGB32:
        case QImage::Format_RGB32:
        case QImage::Format_ARGB32_Premultiplied:
            mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
            break;
        case QImage::Format_RGB888:
            mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
            //mat = cv::Mat(image.height(), image.width(), CV_8UC3, (uchar*)image.bits(), image.bytesPerLine());
            cv::cvtColor(mat, mat, CV_BGR2RGB);
            break;

        case QImage::Format_Grayscale8:
            mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
            break;
        defualt:
            std::cout << "[QImageToMat] no format found" << std::endl;
            break;
    }
    return mat;
}

Vision::Vision() : color_table_filename("/home/gisen/ros/src/opr_ros/opr_vision/color_table.cnf"), terminated(false), it_(nh_)
{
    constexpr int default_view_img_width = 320;
    constexpr int default_view_img_height = 240;
    constexpr int default_grab_img_width = 640;
    constexpr int default_grab_img_height = 480;
    //chainer_wrapper::initialize_wl_detector("predict_wl.py");
    object_detector = std::make_unique<DetectorColorTable>(default_grab_img_width, default_grab_img_height);
//    object_detector = std::make_unique<DetectorWL>(default_grab_img_width, default_grab_img_height);
    img = cv::Mat(default_grab_img_height, default_grab_img_width, CV_8UC3);
    view_img = cv::Mat(default_view_img_height, default_view_img_width, CV_8UC3);
    object_detector->loadColorTable(color_table_filename);

    image_sub_ = it_.subscribe("/camera_sensor/image_raw", 1, &Vision::imageCb, this);
    image_pub_ = it_.advertise("/test/output_vid", 1);
    save_server_ = nh_.advertiseService("/vision/save_color_table", &Vision::saveColorTable, this);
    load_server_ = nh_.advertiseService("/vision/load_color_table", &Vision::loadColorTable, this);
    clear_server_ = nh_.advertiseService("/vision/clear_color_table", &Vision::clearColorTable, this);
    set_color_sub_ = nh_.subscribe("/vision/set_color_table", 1, &Vision::setColorTable, this);
    cv::namedWindow(NAME);
    // cv::namedWindow(NAME2);
    // cv::namedWindow(NAME3);
}

Vision::~Vision()
{
    //image_grabber->stopCamera();
    terminated = true;
    //th.join();
    cv::destroyWindow(NAME);
    // cv::destroyWindow(NAME2);
    // cv::destroyWindow(NAME3);
}

bool Vision::loadColorTable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    object_detector->loadColorTable(color_table_filename);
    ROS_INFO("load color table");
    return true;
}

bool Vision::saveColorTable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    object_detector->saveColorTable(color_table_filename);
    ROS_INFO("save color table");
    return true;
}

bool Vision::clearColorTable(opr_msgs::Color::Request &req, opr_msgs::Color::Response &res)
{
    object_detector->resetColorTable(req.color);
    ROS_INFO("clear color table");
    return true;
}

void Vision::setColorTable(const opr_msgs::SetColor::ConstPtr &msg)
{
    // TODO check mode, rgb to ycbcr ?
    int cb = msg->r;
    int cr = msg->g;
    int y = msg->b;
    int margin = msg->margin;
    unsigned short color_short = static_cast<unsigned short>(msg->color);
    if(msg->clearmode == false)
        /* set color */
        object_detector->setColorTable(y, cb, cr, margin, color_short);
    else
        /* clear color */
        object_detector->clearColorTable(y, cb, cr, margin, color_short);
}

void Vision::setLensType(const int lens_type)
{
    //image_grabber->setLensType(lens_type);
}

std::vector<Pos2D> Vision::getDetectedObject(Object_T obj, Object_Ownership_T ownership)
{
    // std::lock_guard<std::mutex> lock(mutex_objects);
    std::vector<Pos2D> ret;
    if(ownership == OWNERSHIP_ANY) {
        for(object_pos p : objects) {
            if(p.type == obj) {
                Pos2D pos;
                pos.x = p.x;
                pos.y = p.y;
                ret.push_back(pos);
            }
        }
    } else {
        for(object_pos p : objects) {
            if(p.type == obj && p.ownership == ownership) {
                Pos2D pos;
                pos.x = p.x;
                pos.y = p.y;
                ret.push_back(pos);
            }
        }
    }
    return ret;
}

int Vision::getWhiteLinePos(std::vector<NPos2D> &whitelines)
{
    // std::lock_guard<std::mutex> lock(mutex_whiteline);
    whitelines.clear();
    for(object_pos pos : white_line) {
        NPos2D p(pos.x, pos.y);
        whitelines.push_back(p);
    }
    return whitelines.size();
}

void Vision::getImageInfo(int &width, int &height, int &bpp)
{
    width = view_img.cols;
    height = view_img.rows;
    bpp = view_img.channels();
}

void Vision::getImageInfo(const IplImage *ipl_img, int &width, int &height, int &bpp)
{
    width = ipl_img->width;
    height = ipl_img->height;
    bpp = ipl_img->nChannels;
}

void Vision::getImageData(char *data)
{
    // std::unique_lock<std::mutex> lock(mutex_lock);
    cv::resize(img, view_img, view_img.size());
    // lock.unlock();
    const std::size_t data_len = view_img.rows * view_img.cols * view_img.channels();
    memcpy(data, view_img.data, data_len);
}

void Vision::getColorResultData(char *data)
{
    // std::unique_lock<std::mutex> lock(mutex_lock);
    cv::Mat result(img.rows, img.cols, CV_16UC1);
    // lock.unlock();
    object_detector->getColorResultImageData(result);
    cv::Mat resized_result;
    cv::resize(result, resized_result, cv::Size(view_img.cols, view_img.rows), 0, 0, cv::INTER_NEAREST);
    unsigned short *d = reinterpret_cast<unsigned short *>(data);
    const std::size_t data_len = resized_result.rows * resized_result.cols * resized_result.channels();
    for(std::size_t i = 0; i < data_len; i++) {
        d[i] = resized_result.data[i];
    }
}

void Vision::getCameraSettings(bool &bauto, int &shutter, int &gain, int &whitebalance)
{
    //image_grabber->getCameraSettings(bauto, shutter, gain, whitebalance);
}

void Vision::setCameraSettings(bool bauto, int shutter, int gain, int whitebalance)
{
    //image_grabber->setCameraSettings(bauto, shutter, gain, whitebalance);
}

void Vision::readCameraSettingFile(void)
{
    //image_grabber->readCameraSettingFile();
}

void Vision::writeCameraSettingFile(void)
{
    //image_grabber->writeCameraSettingFile();
}

void Vision::setResolution(const int reso)
{
    int width, height;
    switch(reso) {
    case 0:
        width = 320;
        height = 240;
        break;
    case 1:
        width = 640;
        height = 480;
        break;
    default:
        width = 320;
        height = 240;
        break;
    }
    view_img = cv::Mat(height, width, CV_8UC3);
}

int Vision::getBallParticle(std::vector<struct ball_particle_T> &ball_particles)
{
    ball_particles.clear();
    std::vector<Particle> pars = object_detector->getBallParticle();
    for(Particle p : pars) {
        ball_particle_T par(p.x, p.y, p.radius, p.score);
        ball_particles.push_back(par);
    }
    return ball_particles.size();
}

std::vector<struct BoundingBox> Vision::getBallBoundingBox(void)
{
    return object_detector->getBallBoundingBox();
}

std::vector<struct BoundingBox> Vision::getGoalBoundingBox(void)
{
    return object_detector->getGoalBoundingBox();
}

std::vector<struct BoundingBox> Vision::getAlliedRobotBoundingBox(void)
{
    return object_detector->getAlliedRobotBoundingBox();
}

std::vector<struct BoundingBox> Vision::getEnemyRobotBoundingBox(void)
{
    return object_detector->getEnemyRobotBoundingBox();
}

void Vision::getUpdatedImage(IplImage **ret_img)
{
    // std::lock_guard<std::mutex> lock(mutex_lock);
    IplImage i = img;
    ipl = cvCloneImage(&i);
    *ret_img = ipl;
}

int Vision::getWhiteLinePosFromImage(std::vector<NPos2D> &wlmap, const IplImage *wlimg)
{
    wlmap.clear();
    // TODO: implementation
    return 0;
}

bool Vision::setUseYOLO(bool enable, std::string cfg, std::string weights, std::vector<int> thresholds)
{
    if(enable) 
        object_detector->setupYOLO(cfg, weights, thresholds);
    return enable;
}

std::string Vision::detectQRcode(void)
{
    std::string qr_code("nothing");
    zbar::ImageScanner scanner;
    cv::Mat grey;
    // std::unique_lock<std::mutex> lock(mutex_lock);
    cv::cvtColor(img, grey, CV_BGR2GRAY);
    unsigned char *raw = grey.data;
    const int width = img.cols;
    const int height = img.rows;
    // lock.unlock();
    zbar::Image image(width, height, "Y800", raw, width * height);
    [[maybe_unused]] const int n = scanner.scan(image);
    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        qr_code = symbol->get_data();
    }
    return std::string(qr_code);
}

void Vision::update(void)
{
    //try {
        //std::unique_lock<std::mutex> lock_img(mutex_lock);
        //const int error = image_grabber->getImage(img);
        //lock_img.unlock();
        /*
        if(error) {
            {
                std::unique_lock<std::mutex> lock_img(mutex_lock);
                img = cv::Mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            }
            {
                std::unique_lock<std::mutex> lock_whiteline(mutex_whiteline);
                std::unique_lock<std::mutex> lock_objects(mutex_objects);
                white_line.clear();
                objects.clear();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cerr << "[vision] Capture error. Check camera connection." << std::endl;
            throw std::string("capture error");
        }
        lock_img.lock();
        object_detector->setImage(img);
        image_writer.saveCapturedImage(img);
        lock_img.unlock();
        std::unique_lock<std::mutex> lock_whiteline(mutex_whiteline);
        std::unique_lock<std::mutex> lock_objects(mutex_objects);
        */
    //} catch(std::string err) {
        //std::cerr << err << std::endl;
    //}
    
#if 0
    // extract image color data (update_hplstatus)
    int width, height, bpp;
    getImageInfo(width, height, bpp);
    char *data = new char[width * height * bpp];
    getImageData(data);
    getColorResultData(data);
    bpp = sizeof(unsigned short);

    // update color (showColorResult)
    int color = 1; //TODO determine which color to check

    QImage currentImage_rgb(width, height, QImage::Format_RGB888);
    memcpy(currentImage_rgb.scanLine(0), &data[0], width * height * bpp);

    QImage img(width, height, QImage::Format_RGB888);
    unsigned short *p = (unsigned short *)&data[0];
    unsigned char *q = img.scanLine(0);
    unsigned char *rgb = currentImage_rgb.scanLine(0);
    for(int i = 0; i < width * height; i++){
        unsigned short c = *p++;
        unsigned char  b = *rgb++;
        unsigned char  g = *rgb++;
        unsigned char  r = *rgb++;
        if (c & color){
            if(1){
                *q++ = r;
                *q++ = g;
                *q++ = b;
            }
            else{
                *q++ = 255;
                *q++ = 255;
                *q++ = 255;
            }
        } else{
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
        }
    }

    // convert QImage to OpenCV to sensor_msgs/Image
    cv::Mat temp(img.height(), img.width(), CV_8UC3, (uchar*)img.bits(), img.bytesPerLine());
    cv::Mat mat;
    cvtColor(temp, mat, CV_BGR2RGB);
#endif

    //get whitelines
#if 0
    std::vector<NPos2D> whitelines;
    int num_whitelines = getWhiteLinePos(whitelines);

    //visualize whitelines
    cv::Mat result(img.rows, img.cols, CV_16UC1);
    object_detector->getColorResultImageData(result);
#endif
}

#if 0
## update_hplstatus
vision.getImageInfo(w, h, bpp)
vision.getImageData(buf)
status_mgr.setImage(w, h, bpp, buf)
vision.getColorResultData(buf)
status_mgr.setColorResultData(w, h, bpp, buf)
delete [] buf;

detectedObjs
if ballpos_local
    ...
    detectedObjs.pushback()
if goal
    ...

if num_whitelines
    ...
status_mgr.setVisibleObjects(detectedObjs)
status_mgr.setSelfPos(..
status_mgr.setJointAngles
pose.getParticle
status_mgr.setMCL
###

## showColorResult
int color = 1 << ui.comboColor->currentIndex()
int width = status.colresult.width
int height = status.colresult.height
QImage img(width, height, QImage::Format_RGB888)
unsigned short *p = (unsigned short *)&status.colresult.data[0];
unsigned char *q = img.scanLine(0);
unsigned char *rgb = currentImage_rgb.scanLine(0);
int num = width * height;
float score = 0.0f;
for(int i = 0; i < num; i ++){
    unsigned short c = *p++;
    if (c & color){
        *q++ = 255; *q++ = 255; *q++ = 255;
    } else {
        *q++ =   0; *q++ =   0; *q++ =   0;
    }
}
#endif

void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    
    img = cv_ptr->image;
    object_detector->setImage(img);
    object_detector->getObjects(objects, white_line);

    // cv::Mat result(img.rows, img.cols, CV_16UC1);
    // // cv::Mat result(img.rows, img.cols, CV_8UC3);
    // // cv::resize(img, img, cv::Size(320, 240));
    // cv::imshow(NAME2, img);
    // object_detector->getColorResultImageData(result);
    // cv::Mat resized_result;
    // cv::resize(result, resized_result, cv::Size(view_img.cols, view_img.rows), 0, 0, cv::INTER_NEAREST);
    //cv::imwrite("/home/gisen/result.png", result); //img.save("/home/gisen/test.png");


    // extract image color data (update_hplstatus)
    int width, height, bpp;
    getImageInfo(width, height, bpp);
    char *buf = new char[width * height * bpp];
    getImageData(buf);
    // getColorResultData(buf); //TODO: something is happenning in this function, without this its weird, with it nothing appears
    
    // set color commnad + set object detected command
    // object_detector->setColorTable(145, 54, 34, 5, COLOR_GREEN);
    // object_detector->setColorTable(235, 128, 128, 5, COLOR_WHITE);
   
    // convert buf in setColorResultImage
    bpp = sizeof(unsigned short);
    int imsize = width * height * bpp;
    std::vector<std::byte> data;
    data.resize(imsize);
    memcpy(&data[0], buf, imsize);

    // update color (showColorResult)
    int color = 1 << COLOR_WHITE; //TODO determine which color to check
    QImage img(width, height, QImage::Format_RGB888);
    unsigned short *p = (unsigned short *)&data[0];
    unsigned char *q = img.scanLine(0);
    int num = width * height;
    for(int i = 0; i < num; i++){
        unsigned short c = *p++;
        if (c & color){
            *q++ = 255; *q++ = 255; *q++ = 255;
        } else{
            *q++ = 0; *q++ = 0; *q++ = 0;
        }
    }
    // img.save("/home/gisen/test.png");

    // convert QImage to OpenCV to sensor_msgs/Image
    cv::Mat result2 = QImageToMat(img);
    cv::imshow(NAME, result2);
    cv::waitKey(1);
    delete[] buf;
    // std_msgs::Header header;
    // cv_bridge::CvImage img_bridge;
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, result2);
    // image_pub_.publish(img_bridge.toImageMsg());
}


int main(int argc, char **argv){
    ros::init(argc, argv, "opr_vision");
    Vision vision;
   
    ros::spin();
    //ros::Rate rate(10);
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    //while(ros::ok()){
    //    vision.update();
    //    rate.sleep();
    //}
    //spinner.stop();
    
    return 0;
}

