#include <mutex>
#include <algorithm>

#include "detector_wl.h"
#include "detector_tool.h"

static void ipl_into_image(IplImage *src, image im)
{
	unsigned char *data = (unsigned char *)src->imageData;
	int h = src->height;
	int w = src->width;
	int c = src->nChannels;
	int step = src->widthStep;

	for(int i = 0; i < h; i++){
		for(int k = 0; k < c; k++){
			for(int j = 0; j < w; j++){
				im.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.0;
			}
		}
	}
}

static image ipl_to_image(IplImage *src)
{
	int h = src->height;
	int w = src->width;
	int c = src->nChannels;
	image out = make_image(w, h, c);
	ipl_into_image(src, out);
	return out;
}

static bool isOverlap(const struct BoundingBox box1, const struct BoundingBox box2)
{
	const float x1 = std::max(box1.getLeft(), box2.getLeft());
	const float y1 = std::max(box1.getTop(), box2.getTop());
	const float x2 = std::min(box1.getRight(), box2.getRight());
	const float y2 = std::min(box1.getBottom(), box2.getBottom());
	if(x1 < x2 && y1 < y2) {
		return true;
	} else {
		return false;
	}
}

/*
 * calculate IoU(Intersection over Union).
 * ref: https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
 */
static float IoU(const struct BoundingBox b1, const struct BoundingBox b2)
{
	if(!isOverlap(b1, b2))
		return 0.0;

	const auto area_of_union =
		(b1.getRight() - b1.getLeft()) * (b1.getBottom() - b1.getTop()) +
		(b2.getRight() - b2.getLeft()) * (b2.getBottom() - b2.getTop());
	const auto x1 = std::max(b1.getLeft(), b2.getLeft());
	const auto x2 = std::min(b1.getRight(), b2.getRight());
	const auto y1 = std::max(b1.getTop(), b2.getTop());
	const auto y2 = std::min(b1.getBottom(), b2.getBottom());
	const auto area_of_intersection = (x2 - x1) * (y2 - y1);
	const auto iou = area_of_intersection / (area_of_union / 2.0);
	return iou;
}

extern std::mutex label_img_mutex_lock;
extern std::mutex label_img_for_visualize_mutex_lock;

DetectorWL::DetectorWL(const int w, const int h) : ObjectDetector(w, h), object_kind_num(4), score_threshold(object_kind_num, 0.0), yolo_ready(false)
{
	constexpr int predict_image_width = 320;
	constexpr int predict_image_height = 240;
	deep_learning.setImageSize(predict_image_width, predict_image_height);
#ifdef VREP_SIMULATOR
	setColorTable(88 , 78 , 65 , 1, COLOR_GREEN);
	setColorTable(255, 128, 128, 1, COLOR_WHITE);
	setColorTable(18 , 128, 128, 5, COLOR_BLACK);
#endif
}

DetectorWL::~DetectorWL()
{
}

void DetectorWL::getObjects(std::vector<object_pos> &objects, std::vector<object_pos> &white_line)
{
	objects.clear();
	white_line.clear();

	std::unique_lock<std::mutex> lock_label_img(label_img_mutex_lock);
	detectBall(objects);
	getLabelingImage(label_img);

	std::vector<int> field_edge(label_img.cols, 0);
	std::vector<int> obstacle_pixel(field_edge);
	DetectorTool::detectFieldEdge(label_img, field_edge, obstacle_pixel);
	DetectorTool::eraseWhiteLineOutsideField(label_img, field_edge);
	DetectorTool::showFieldEdge(label_img, field_edge);

	DetectorTool::detectWhiteLine(label_img, white_line);

	std::lock_guard<std::mutex> lock_label_img_for_visualize(label_img_for_visualize_mutex_lock);
	label_img_for_visualize = label_img.clone();
}

/*
 * Detect ball, goal poles, allied robots and enemy robots with YOLO
 */
void DetectorWL::detectBall(std::vector<object_pos> &objects)
{
	if(!yolo_ready)
		return;

	cv::Mat rgb_img, resized_img;
	cv::cvtColor(img, rgb_img, CV_YCrCb2BGR);
	cv::resize(rgb_img, resized_img, cv::Size(net->w, net->h));
	IplImage ipl_i = resized_img;
	image im = ipl_to_image(&ipl_i);
	float *X = im.data;
	network_predict(net, X);
	int nboxes = 0;
	float min_thresh = 1.0;
	for(int i = 0; i < LABEL_TOTAL_NUM; i++) {
		if(score_threshold[i] < min_thresh)
			min_thresh = score_threshold[i];
	}
	const float hier_thresh = min_thresh;
	detection *dets = get_network_boxes(net, im.w, im.h, min_thresh, hier_thresh, 0, 1, &nboxes);
	layer l = net->layers[net->n - 1];
	float nms = 0.45;
	if(nms)
		do_nms_sort(dets, nboxes, l.classes, nms);

	ball_box.clear();
	goal_post_box.clear();
	allied_robot_box.clear();
	enemy_robot_box.clear();
	for(int i = 0; i < nboxes; i++) {
		if(dets[i].prob[LABEL_BALL] > score_threshold[LABEL_BALL]) {
			const float score = dets[i].prob[LABEL_BALL];
			struct BoundingBox b(dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h, score);
			ball_box.push_back(b);
		}
		if(dets[i].prob[LABEL_GOAL] > score_threshold[LABEL_GOAL]) {
			const float score = dets[i].prob[LABEL_GOAL];
			struct BoundingBox b(dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h, score);
			goal_post_box.push_back(b);
		}
		if(dets[i].prob[LABEL_ALLIED_ROBOT] > score_threshold[LABEL_ALLIED_ROBOT]) {
			const float score = dets[i].prob[LABEL_ALLIED_ROBOT];
			struct BoundingBox b(dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h, score);
			allied_robot_box.push_back(b);
		}
		if(dets[i].prob[LABEL_ENEMY_ROBOT] > score_threshold[LABEL_ENEMY_ROBOT]) {
			const float score = dets[i].prob[LABEL_ENEMY_ROBOT];
			struct BoundingBox b(dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h, score);
			enemy_robot_box.push_back(b);
		}
	}
	if(ball_box.size()) {
		std::sort(ball_box.begin(), ball_box.end());
		constexpr std::size_t max_ball_num = 1;
		if(ball_box.size() > max_ball_num) {
			ball_box.erase(ball_box.begin() + max_ball_num, ball_box.end());
		}
		for(auto b : ball_box) {
			const int x = static_cast<int>(b.x * img.cols);
			const int y = static_cast<int>(b.y * img.rows);
			objects.push_back(object_pos(x, y, OBJECT_BALL));
		}
	}
	if(goal_post_box.size()) {
		std::sort(goal_post_box.begin(), goal_post_box.end());
		// erase overlapped bounding boxes of goal pole.
		std::vector<BoundingBox> tmp_goal_box = goal_post_box;
		goal_post_box.clear();
		goal_post_box.push_back(tmp_goal_box[0]);
		constexpr std::size_t max_goal_post_num = 2;
		for(std::size_t i = 1; i < tmp_goal_box.size(); i++) {
			for(std::size_t j = 0; j < goal_post_box.size(); j++) {
				if(!isOverlap(goal_post_box[j], tmp_goal_box[i])) {
					goal_post_box.push_back(tmp_goal_box[i]);
					if(goal_post_box.size() >= max_goal_post_num)
						goto abort_loop_max_goal_pole_num;
				}
			}
		}
		abort_loop_max_goal_pole_num: ;
		for(auto b : goal_post_box) {
			const int x = static_cast<int>(b.x * img.cols);
			const int lower_y = std::min(static_cast<int>(b.getBottom() * img.rows), height - 1);
			objects.push_back(object_pos(x, lower_y, OBJECT_GOAL_POST));
		}
	}
	if(allied_robot_box.size()) {
		std::sort(allied_robot_box.begin(), allied_robot_box.end());
		std::vector<BoundingBox> tmp_allied_robot_box = allied_robot_box;
		allied_robot_box.clear();
		allied_robot_box.push_back(tmp_allied_robot_box[0]);
		constexpr std::size_t max_allied_robot_num = 3;
		constexpr float threshold_iou = 0.5;
		for(std::size_t i = 1; i < tmp_allied_robot_box.size(); i++) {
			for(std::size_t j = 0; j < allied_robot_box.size(); j++) {
				if(IoU(allied_robot_box[j], tmp_allied_robot_box[i]) < threshold_iou)
					allied_robot_box.push_back(tmp_allied_robot_box[i]);
					if(allied_robot_box.size() >= max_allied_robot_num)
						goto abort_loop_max_allied_robot_num;
			}
		}
		abort_loop_max_allied_robot_num: ;
		for(auto b : allied_robot_box) {
			const int x = static_cast<int>(b.x * img.cols);
			const int lower_y = std::min(static_cast<int>(b.getBottom() * img.rows), height - 1);
			objects.push_back(object_pos(x, lower_y, OBJECT_ROBOT, OWNERSHIP_OURS));
		}
	}
	if(enemy_robot_box.size()) {
		std::sort(enemy_robot_box.begin(), enemy_robot_box.end());
		std::vector<BoundingBox> tmp_enemy_robot_box = enemy_robot_box;
		enemy_robot_box.clear();
		enemy_robot_box.push_back(tmp_enemy_robot_box[0]);
		constexpr std::size_t max_enemy_robot_num = 4;
		constexpr float threshold_iou = 0.5;
		for(std::size_t i = 1; i < tmp_enemy_robot_box.size(); i++) {
			for(std::size_t j = 0; j < enemy_robot_box.size(); j++) {
				if(IoU(enemy_robot_box[j], tmp_enemy_robot_box[i]) < threshold_iou)
					enemy_robot_box.push_back(tmp_enemy_robot_box[i]);
					if(enemy_robot_box.size() >= max_enemy_robot_num)
						goto abort_loop_max_enemy_robot_num;
			}
		}
		abort_loop_max_enemy_robot_num: ;
		for(auto b : enemy_robot_box) {
			const int x = static_cast<int>(b.x * img.cols);
			const int lower_y = std::min(static_cast<int>(b.getBottom() * img.rows), height - 1);
			objects.push_back(object_pos(x, lower_y, OBJECT_ROBOT, OWNERSHIP_THEIRS));
		}
	}

	free_detections(dets, nboxes);
	free_image(im);
}

void DetectorWL::getLabelingImage(cv::Mat &labeling_image)
{
	cv::Mat median_image = cv::Mat(img.rows, img.cols, CV_16UC1);
	constexpr int kernel_size = 3;
	cv::medianBlur(img, median_image, kernel_size);
	labeling_image = cv::Mat(img.rows, img.cols, CV_16UC1);
	color_table.apply(median_image, labeling_image);

	// Detect white line by deep learning
	// labeling image size: 640x480
	// Network input image size: 320x240
	cv::Mat rgb_img, wl_image;
	cv::cvtColor(img, rgb_img, CV_YCrCb2RGB);
	cv::resize(rgb_img, rgb_img, cv::Size(320, 240));
	wl_image = deep_learning.predict(rgb_img);

	const unsigned short white = (1 << COLOR_WHITE);
	for(int y = 0; y < labeling_image.rows; y++) {
		for(int x = 0; x < labeling_image.cols; x++) {
			if(wl_image.data[(y / 2) * wl_image.cols + (x / 2)] != 0) {
				labeling_image.data[y * labeling_image.cols + x] |= white;
			} else {
				labeling_image.data[y * labeling_image.cols + x] &= ~white;
			}
		}
	}
}

void DetectorWL::setColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
	color_table.setColor(y, cb, cr, object_type, margin);
}

void DetectorWL::clearColorTable(int y, int cb, int cr, int margin, unsigned short object_type)
{
	color_table.clearColor(y, cb, cr, object_type, margin);
}

void DetectorWL::resetColorTable(unsigned short object_type)
{
	color_table.resetColor(object_type);
}

void DetectorWL::loadColorTable(std::string filename)
{
	try {
		color_table.loadColorTable(filename);
	} catch(...) {
		std::cerr << "Failed to load color table" << std::endl;
	}
}

void DetectorWL::saveColorTable(std::string filename)
{
	try {
		color_table.saveColorTable(filename);
	} catch(...) {
		std::cerr << "Failed to save color table" << std::endl;
	}
}

bool DetectorWL::setupYOLO(std::string config_file, std::string weight_file, std::vector<int> thresholds)
{
	score_threshold.clear();
	std::transform(thresholds.begin(), thresholds.end(), score_threshold.begin(), [](const int v){ return static_cast<float>(v / 100.0); });
	config_file_name = config_file;
	weight_file_name = weight_file;
	net = load_network(const_cast<char *>(config_file_name.c_str()), const_cast<char *>(weight_file_name.c_str()), 0);
	set_batch_network(net, 1);
	yolo_ready = true;
	return true;
}

std::vector<struct BoundingBox> DetectorWL::getBallBoundingBox(void)
{
	return ball_box;
}

std::vector<struct BoundingBox> DetectorWL::getGoalBoundingBox(void)
{
	return goal_post_box;
}

std::vector<struct BoundingBox> DetectorWL::getAlliedRobotBoundingBox(void)
{
	return allied_robot_box;
}

std::vector<struct BoundingBox> DetectorWL::getEnemyRobotBoundingBox(void)
{
	return enemy_robot_box;
}

