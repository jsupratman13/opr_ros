#include <cstdlib>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>

#ifdef __cpp_lib_filesystem
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <zbar.h>

#include "vision.h"
// #ifdef VREP_SIMULATOR
// #include "image_grabber/image_grabber_vrep.h"
// #elif defined WEBOTS_VHSC
// #include "image_grabber/image_grabber_webots.h"
// #elif defined WEBOTS_DIRECT
// #include "image_grabber/image_grabber_webots_direct.h"
// #else
// #include "image_grabber/image_grabber_v4l.h"
// #endif
#include "image_grabber/image_grabber_ros.h"

// #ifdef GPU
// #include "detector/detector_wl.h"
// #else
// #include "detector/detector_color_table.h"
// #endif
#include "detector/detector_color_table.h"

class ImageWriter
{
public:
    ImageWriter();
    ~ImageWriter();
    void createCapturedImageDirectory();
    void saveCapturedImage(cv::Mat);
private:
    bool enable_capture;
    int image_count;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_capture_time;
    std::string capture_save_path;
};

ImageWriter::ImageWriter() : enable_capture(false), image_count(0)
{
    const char *env_capture = std::getenv("CAPTURE");
    if(env_capture && env_capture[0] == '1') {
        createCapturedImageDirectory();
        enable_capture = true;
        last_capture_time = std::chrono::high_resolution_clock::now();
    }
}

ImageWriter::~ImageWriter()
{
}

void ImageWriter::createCapturedImageDirectory(void)
{
    std::string capture_dir("game_images");
    if(!fs::exists(capture_dir)) {
        fs::create_directory(capture_dir);
    }
    std::stringstream ss_capture;
    auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ss_capture << capture_dir << "/" << std::put_time(std::localtime(&now_t), "%Y%m%dT%H%M%S");
    capture_save_path = ss_capture.str();
    fs::create_directory(capture_save_path);
}

void ImageWriter::saveCapturedImage(cv::Mat img)
{
    if(enable_capture) {
        const auto capture_interval = std::chrono::seconds(1);
        if(std::chrono::high_resolution_clock::now() - last_capture_time > capture_interval) {
            std::stringstream ss;
            ss << capture_save_path << "/" << std::setw(6) << std::setfill('0') << image_count << ".jpg";
            image_count++;
            cv::Mat save_img;
            cv::cvtColor(img, save_img, cv::COLOR_YCrCb2BGR);
            cv::imwrite(ss.str().c_str(), save_img);
            last_capture_time = std::chrono::high_resolution_clock::now();
        }
    }
}

static std::mutex mutex_lock;
static std::mutex mutex_whiteline;
static std::mutex mutex_objects;

namespace CitBrains {
    Vision::Vision(const int id) : color_table_filename("/home/gisen/ros/src/opr_ros/opr_vision/color_table.cnf"), terminated(false)
    {
        constexpr int default_view_img_width = 320;
        constexpr int default_view_img_height = 240;
        constexpr int default_grab_img_width = 640;
        constexpr int default_grab_img_height = 480;
        //chainer_wrapper::initialize_wl_detector("predict_wl.py");
// #ifdef VREP_SIMULATOR
//         image_grabber = std::make_unique<ImageGrabberVrep>(id, default_grab_img_width, default_grab_img_height);
// #elif defined WEBOTS_VHSC
//         image_grabber = std::make_unique<ImageGrabberWebots>(id, default_grab_img_width, default_grab_img_height);
// #elif defined WEBOTS_DIRECT
//         image_grabber = std::make_unique<ImageGrabberWebots>(id, default_grab_img_width, default_grab_img_height);
// #else
//         image_grabber = std::make_unique<ImageGrabberV4l>(id, default_grab_img_width, default_grab_img_height);
// #endif
        image_grabber = std::make_unique<ImageGrabberROS>(id, default_grab_img_width, default_grab_img_height);

// #ifdef GPU
//         object_detector = std::make_unique<DetectorWL>(default_grab_img_width, default_grab_img_height);
// #else
//         object_detector = std::make_unique<DetectorColorTable>(default_grab_img_width, default_grab_img_height);
// #endif
        object_detector = std::make_unique<DetectorColorTable>(default_grab_img_width, default_grab_img_height);

        img = cv::Mat(default_grab_img_height, default_grab_img_width, CV_8UC3);
        view_img = cv::Mat(default_view_img_height, default_view_img_width, CV_8UC3);
        object_detector->loadColorTable(color_table_filename);
        image_grabber->startCamera();
        th = std::thread(&CitBrains::Vision::threadRun, this);
    }

    Vision::~Vision()
    {
        image_grabber->stopCamera();
        terminated = true;
        th.join();
    }

    void Vision::setLensType(const int lens_type)
    {
        image_grabber->setLensType(lens_type);
    }

    std::vector<Pos2D> Vision::getDetectedObject(Object_T obj, Object_Ownership_T ownership)
    {
        std::lock_guard<std::mutex> lock(mutex_objects);
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
        std::lock_guard<std::mutex> lock(mutex_whiteline);
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

    void Vision::getImageInfo(const cv::Mat _img, int &width, int &height, int &bpp)
    {
        width = _img.cols;
        height = _img.rows;
        bpp = _img.channels();
    }

    void Vision::getImageData(char *data)
    {
        std::unique_lock<std::mutex> lock(mutex_lock);
        cv::resize(img, view_img, view_img.size());
        lock.unlock();
        const std::size_t data_len = view_img.rows * view_img.cols * view_img.channels();
        memcpy(data, view_img.data, data_len);
    }

    void Vision::getColorResultData(char *data)
    {
        std::unique_lock<std::mutex> lock(mutex_lock);
        cv::Mat result(img.rows, img.cols, CV_16UC1);
        lock.unlock();
        object_detector->getColorResultImageData(result);
        cv::Mat resized_result;
        cv::resize(result, resized_result, cv::Size(view_img.cols, view_img.rows), 0, 0, cv::INTER_NEAREST);
        unsigned short *d = reinterpret_cast<unsigned short *>(data);
        const std::size_t data_len = resized_result.rows * resized_result.cols * resized_result.channels();
        for(std::size_t i = 0; i < data_len; i++) {
            d[i] = resized_result.data[i];
        }
    }

    void Vision::setColorTable([[maybe_unused]] const int camno, int cb, int cr, int y, int margin, int color, bool bclear)
    {
        unsigned short color_short = static_cast<unsigned short>(color);
        if(bclear == false)
            /* set color */
            object_detector->setColorTable(y, cb, cr, margin, color_short);
        else
            /* clear color */
            object_detector->clearColorTable(y, cb, cr, margin, color_short);
    }

    void Vision::loadColorTable(void)
    {
        object_detector->loadColorTable(color_table_filename);
    }

    void Vision::saveColorTable(void)
    {
        object_detector->saveColorTable(color_table_filename);
    }

    void Vision::clearColorTable(int object_type)
    {
        object_detector->resetColorTable(object_type);
    }

    void Vision::getCameraSettings(bool &bauto, int &shutter, int &gain, int &whitebalance)
    {
        image_grabber->getCameraSettings(bauto, shutter, gain, whitebalance);
    }

    void Vision::setCameraSettings(bool bauto, int shutter, int gain, int whitebalance)
    {
        image_grabber->setCameraSettings(bauto, shutter, gain, whitebalance);
    }

    void Vision::readCameraSettingFile(void)
    {
        image_grabber->readCameraSettingFile();
    }

    void Vision::writeCameraSettingFile(void)
    {
        image_grabber->writeCameraSettingFile();
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

    void Vision::getUpdatedImage(cv::Mat &ret_img)
    {
        std::lock_guard<std::mutex> lock(mutex_lock);
        ret_img = img;
    }

    int Vision::getWhiteLinePosFromImage(std::vector<NPos2D> &wlmap, const cv::Mat &wlimg)
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
        std::unique_lock<std::mutex> lock(mutex_lock);
        cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);
        unsigned char *raw = grey.data;
        const int width = img.cols;
        const int height = img.rows;
        lock.unlock();
        zbar::Image image(width, height, "Y800", raw, width * height);
        [[maybe_unused]] const int n = scanner.scan(image);
        for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            qr_code = symbol->get_data();
        }
        return std::string(qr_code);
    }

    void Vision::threadRun(void)
    {
        ImageWriter image_writer;
        while(!terminated) {
            try {
                std::unique_lock<std::mutex> lock_img(mutex_lock);
                const int error = image_grabber->getImage(img);
                lock_img.unlock();
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
                object_detector->getObjects(objects, white_line);
            } catch(std::string err) {
                std::cerr << err << std::endl;
            }
        }
    }
};

