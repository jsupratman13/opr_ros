#include <mutex>
#include <algorithm>

#include "detector_color_table.h"
#include "detector_tool.h"

extern std::mutex label_img_mutex_lock;
extern std::mutex label_img_for_visualize_mutex_lock;

DetectorColorTable::DetectorColorTable(const int w, const int h) : ObjectDetector(w, h)
{
	setColorTable(152, 42 , 201, 1, COLOR_BALL); // orange ball
	//setColorTable(7 , 171 , 7 , 5, COLOR_GREEN);
	setColorTable(145 , 54 , 34 , 10, COLOR_GREEN);
	//setColorTable(255, 255, 255, 1, COLOR_WHITE);
	setColorTable(235, 128, 128, 10, COLOR_WHITE);
	setColorTable(18 , 128, 128, 5, COLOR_BLACK);
}

DetectorColorTable::~DetectorColorTable()
{
}

void DetectorColorTable::getObjects(std::vector<object_pos> &objects, std::vector<object_pos> &white_line)
{
	objects.clear();
	white_line.clear();

	std::unique_lock<std::mutex> lock_label_img(label_img_mutex_lock);
	getLabelingImage(label_img);
	DetectorTool::eraseWhiteLineOutsideField(label_img);

	std::vector<int> field_edge(label_img.cols, 0);
	std::vector<int> obstacle_pixel(field_edge);
	DetectorTool::detectFieldEdge(label_img, field_edge, obstacle_pixel);
	DetectorTool::detectObstacle(field_edge, obstacle_pixel, objects);
	DetectorTool::showFieldEdge(label_img, field_edge);

	ball_particle.apply(label_img, objects);
	DetectorTool::detectWhiteLine(label_img, white_line);

	std::lock_guard<std::mutex> lock_label_img_for_visualize(label_img_for_visualize_mutex_lock);
	label_img_for_visualize = label_img.clone();
}

void DetectorColorTable::getLabelingImage(cv::Mat &labeling_image)
{
	cv::Mat median_image = cv::Mat(img.rows, img.cols, CV_16UC1);
	constexpr int kernel_size = 3;
	cv::medianBlur(img, median_image, kernel_size);
	labeling_image = cv::Mat(img.rows, img.cols, CV_16UC1);
	try {
		color_table.apply(median_image, labeling_image);
	} catch(...) {
		for(std::size_t i = 0; i < labeling_image.cols * labeling_image.rows * labeling_image.channels(); i++) {
			labeling_image.data[i] = 0x00;
		}
	}
}

void DetectorColorTable::setColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
	color_table.setColor(y, cb, cr, object_type, margin);
}

void DetectorColorTable::clearColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
	color_table.clearColor(y, cb, cr, object_type, margin);
}

void DetectorColorTable::resetColorTable(unsigned short object_type)
{
	color_table.resetColor(object_type);
}

void DetectorColorTable::loadColorTable(std::string filename)
{
	try {
		color_table.loadColorTable(filename);
	} catch(...) {
		std::cerr << "Failed to load color table" << std::endl;
	}
}

void DetectorColorTable::saveColorTable(std::string filename)
{
	try {
		color_table.saveColorTable(filename);
	} catch(...) {
		std::cerr << "Failed to save color table" << std::endl;
	}
}

std::vector<Particle> DetectorColorTable::getBallParticle(void)
{
	std::vector<Particle> ret(ball_particle.getParticle());
	return ret;
}

