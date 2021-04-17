#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../object_types.h"
#include "particle.h"

class ObjectDetector
{
public:
	ObjectDetector(const int, const int);
	virtual ~ObjectDetector();
	virtual void getObjects(std::vector<object_pos> &, std::vector<object_pos> &) = 0;
	virtual void setColorTable(int, int, int, int, unsigned short);
	virtual void clearColorTable(int, int, int, int, unsigned short);
	virtual void resetColorTable(unsigned short);
	virtual void loadColorTable(std::string);
	virtual void saveColorTable(std::string);
    virtual void getLabelingImage(cv::Mat &) = 0;
	void setImage(cv::Mat);
	void setImageSize(const int, const int);
	virtual std::vector<Particle> getBallParticle(void);
	virtual std::vector<struct BoundingBox> getBallBoundingBox(void);
	virtual std::vector<struct BoundingBox> getGoalBoundingBox(void);
	virtual std::vector<struct BoundingBox> getAlliedRobotBoundingBox(void);
	virtual std::vector<struct BoundingBox> getEnemyRobotBoundingBox(void);
	virtual bool setupYOLO(std::string, std::string, std::vector<int>);
	void getColorResultImageData(cv::Mat &);
protected:
	cv::Mat img;
	cv::Mat label_img;
	cv::Mat label_img_for_visualize;
	int width;
	int height;
};

#endif // OBJECT_DETECTOR_H

