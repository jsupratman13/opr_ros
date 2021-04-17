#ifndef DETECTOR_FCN_H
#define DETECTOR_FCN_H

#include "object_detector.h"
#include "predict_by_deep_learning.h"
#include "ball_particle.h"

class DetectorFCN : public ObjectDetector
{
public:
	DetectorFCN(const int, const int);
	~DetectorFCN();
	void getObjects(std::vector<object_pos> &, std::vector<object_pos> &) override;
	std::vector<Particle> getBallParticle(void) override;
private:
	DeepLearning deep_learning;
	BallParticle ball_particle;
	void getLabelingImage(cv::Mat &);
};

#endif // DETECTOR_FCN_H

