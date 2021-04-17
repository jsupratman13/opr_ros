#ifndef DETECTOR_COLOR_TABLE_H
#define DETECTOR_COLOR_TABLE_H

#include "object_detector.h"
#include "color_table.h"
#include "ball_particle.h"

class DetectorColorTable : public ObjectDetector
{
public:
	DetectorColorTable(const int, const int);
	~DetectorColorTable();
	void getObjects(std::vector<object_pos> &, std::vector<object_pos> &) override;
	void setColorTable(int, int, int, int, unsigned short) override;
	void clearColorTable(int, int, int, int, unsigned short) override;
	void resetColorTable(unsigned short) override;
	void loadColorTable(std::string) override;
	void saveColorTable(std::string) override;
	std::vector<Particle> getBallParticle(void) override;
private:
	ColorTable color_table;
	BallParticle ball_particle;
	void getLabelingImage(cv::Mat &);
	double calcParticleScore(int, int, int);
	void detectBallByParticle(std::vector<object_pos> &);
};

#endif // DETECTOR_COLOR_TABLE_H

