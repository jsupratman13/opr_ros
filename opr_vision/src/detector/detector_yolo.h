#ifndef DETECTOR_YOLO_H
#define DETECTOR_YOLO_H

#include "object_detector.h"
#include "color_table.h"
#include "darknet.h"

// YOLO class num
enum {
	LABEL_BALL,
	LABEL_GOAL,
	LABEL_ALLIED_ROBOT,
	LABEL_ENEMY_ROBOT,
	LABEL_TOTAL_NUM,
};

class DetectorYOLO : public ObjectDetector
{
public:
	DetectorYOLO(const int, const int);
	~DetectorYOLO();
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

#endif // DETECTOR_YOLO_H

