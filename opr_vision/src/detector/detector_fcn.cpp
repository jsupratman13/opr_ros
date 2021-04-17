#include <mutex>

#include "detector_fcn.h"
#include "detector_tool.h"

extern std::mutex label_img_mutex_lock;
extern std::mutex label_img_for_visualize_mutex_lock;

DetectorFCN::DetectorFCN(const int w, const int h) : ObjectDetector(w, h)
{
	constexpr int predict_image_width = 256;
	constexpr int predict_image_height = 256;
	deep_learning.setImageSize(predict_image_width, predict_image_height);
}

DetectorFCN::~DetectorFCN()
{
}

void DetectorFCN::getObjects(std::vector<object_pos> &objects, std::vector<object_pos> &white_line)
{
	objects.clear();
	white_line.clear();

	std::unique_lock<std::mutex> lock_label_img(label_img_mutex_lock);
	getLabelingImage(label_img);
	DetectorTool::eraseWhiteLineOutsideField(label_img);

	ball_particle.apply(label_img, objects);
	DetectorTool::detectWhiteLine(label_img, white_line);

	std::lock_guard<std::mutex> lock_label_img_for_visualize(label_img_for_visualize_mutex_lock);
	label_img_for_visualize = label_img.clone();
}

void DetectorFCN::getLabelingImage(cv::Mat &labeling_image)
{
	cv::Mat rgb_img;
	cv::cvtColor(img, rgb_img, CV_YCrCb2RGB);
	labeling_image = deep_learning.predict(rgb_img);
}

std::vector<Particle> DetectorFCN::getBallParticle(void)
{
	std::vector<Particle> ret(ball_particle.getParticle());
	return ret;
}

