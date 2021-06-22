#ifndef VISION_H
#define VISION_H

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "hpl_types.h"
#include "image_grabber/image_grabber.h"
#include "detector/object_detector.h"

namespace CitBrains {
    class Vision
    {
    public:
        Vision(const int);
        ~Vision();
        void setLensType(const int);
        std::vector<Pos2D> getDetectedObject(Object_T, Object_Ownership_T = OWNERSHIP_ANY);
        int getWhiteLinePos(std::vector<NPos2D> &);
        void getImageInfo(int &, int &, int &);
        void getImageInfo(const cv::Mat, int &, int &, int &);
        void getImageData(char *);
        void getColorResultData(char *);
        void setColorTable(const int, int, int, int, int, int, bool);
        void loadColorTable(void);
        void saveColorTable(void);
        void clearColorTable(int);
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
        void getUpdatedImage(cv::Mat &);
        int getWhiteLinePosFromImage(std::vector<NPos2D> &, const cv::Mat &);
        bool setUseYOLO(bool, std::string, std::string, std::vector<int>);
        std::string detectQRcode(void);
    private:
        void threadRun(void);
        std::thread th;
        std::string color_table_filename;
        std::vector<object_pos> objects;
        std::vector<object_pos> white_line;
        cv::Mat img;
        cv::Mat view_img;
        std::unique_ptr<ImageGrabber> image_grabber;
        std::unique_ptr<ObjectDetector> object_detector;
        bool terminated;
    };
};

#endif // VISION_H

