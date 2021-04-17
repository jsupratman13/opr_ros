#ifndef DETECTOR_WL_H
#define DETECTOR_WL_H

#include "object_detector.h"
#include "color_table.h"
#include "predict_by_deep_learning.h"
#include "darknet.h"

// YOLO class num
enum {
	LABEL_BALL,
	LABEL_GOAL,
	LABEL_ALLIED_ROBOT,
	LABEL_ENEMY_ROBOT,
	LABEL_TOTAL_NUM,
};

class DetectorWL : public ObjectDetector
{
public:
	DetectorWL(const int, const int);
	~DetectorWL();
	void getObjects(std::vector<object_pos> &, std::vector<object_pos> &) override;
	std::vector<struct BoundingBox> getBallBoundingBox(void) override;
	std::vector<struct BoundingBox> getGoalBoundingBox(void) override;
	std::vector<struct BoundingBox> getAlliedRobotBoundingBox(void) override;
	std::vector<struct BoundingBox> getEnemyRobotBoundingBox(void) override;
	void setColorTable(int, int, int, int, unsigned short) override;
	void clearColorTable(int, int, int, int, unsigned short) override;
	void resetColorTable(unsigned short) override;
	void loadColorTable(std::string) override;
	void saveColorTable(std::string) override;
	bool setupYOLO(std::string, std::string, std::vector<int>) override;
private:
	DeepLearning deep_learning;
	ColorTable color_table;
	network *net;
	layer layers;
	std::string config_file_name;
	std::string weight_file_name;
	const unsigned int object_kind_num;
	std::vector<float> score_threshold;
	bool yolo_ready;
	std::vector<struct BoundingBox> ball_box;
	std::vector<struct BoundingBox> goal_post_box;
	std::vector<struct BoundingBox> allied_robot_box;
	std::vector<struct BoundingBox> enemy_robot_box;
	void detectBall(std::vector<object_pos> &);
	void getLabelingImage(cv::Mat &);
};

#endif // DETECTOR_WL_H

