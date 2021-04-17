#include <random>
#include <algorithm>

#include "ball_particle.h"

static inline bool isInCircle(const int x, const int y, const int radius)
{
	if(x * x + y * y < radius * radius)
		return true;
	else
		return false;
}

static inline int getBallRadius(const int height, [[maybe_unused]] const int x, const int y)
{
	/*
	 * TODO:
	 * implementation that returns the result calculated based on the camera posture.
	 * maybe need a connection to the posture module.
	 */
	constexpr double dummy_coefficient = 45.0;
	return static_cast<int>((dummy_coefficient * (static_cast<double>(y) / height)));
}

static double calcParticleScore(cv::Mat &label_img, int x, int y, int radius)
{
	int count_ball = 0;
	int count_circle = 0;
	int count_nball = 0;
	int count_ncircle = 0;
	const int margin = radius * 1.2;
	const int min_x = std::max<int>(0, x - margin);
	const int max_x = std::min<int>(label_img.cols - 1, x + margin);
	const int min_y = std::max<int>(0, y - margin);
	const int max_y = std::min<int>(label_img.rows - 1, y + margin);
	std::uniform_int_distribution<> rand_width(min_x, max_x);
	std::uniform_int_distribution<> rand_height(min_y, max_y);
	std::random_device rnd;
	constexpr int check_count = 50; // trade-off between processing time and accuracy.
	for(int i = 0; i < check_count; i++) {
		const int rx = rand_width(rnd);
		const int ry = rand_height(rnd);
		const unsigned short label = label_img.data[(ry * label_img.cols + rx) * label_img.channels()];
		if(isInCircle(rx - x, ry - y, radius)) {
			if(label & (1 << COLOR_BALL)) { // ball color
				count_ball++;
			}
			count_circle++;
		} else {
			if(label & (1 << COLOR_BLACK) || label & (1 << COLOR_GREEN)) { // back-ground color
				count_nball++;
			}
			count_ncircle++;
		}
	}
	double score;
	if(count_circle == 0 || count_ncircle == 0)
		score = 0.0;
	else
		score = (static_cast<double>(count_ball) / static_cast<double>(count_circle)) *
		        (static_cast<double>(count_nball) / static_cast<double>(count_ncircle));
	return score;
}

BallParticle::BallParticle() : particle_num(100), random_particle_num(30), surviving_particle_num(5), particle(particle_num)
{
	// set initial particles at the center of the image.
	constexpr int w = 640; // TODO: get these values from vision module.
	constexpr int h = 480;
	constexpr int center_width = w / 2;
	constexpr int center_height = h / 2;
	Particle p(center_width, center_height);
	for(int i = 0; i < surviving_particle_num; i++) {
		particle[i] = p;
	}
}

BallParticle::~BallParticle()
{
}

void BallParticle::apply(cv::Mat &label_img, std::vector<object_pos> &obj)
{
	particle.resize(particle_num);
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand_width(0, label_img.cols - 1);
	std::uniform_int_distribution<> rand_height(0, label_img.rows - 1);
	const int random_margin = label_img.cols / 40;
	std::uniform_int_distribution<> random_particle_margin(-random_margin, random_margin);
	for(int i = 0; i < surviving_particle_num; i++) {
		particle[i].score = calcParticleScore(label_img, particle[i].x, particle[i].y, particle[i].radius);
	}
	for(int i = surviving_particle_num; i < particle.size(); i++) {
		particle[i].x = rand_width(mt);
		particle[i].y = rand_height(mt);
		if(i < random_particle_num) {
			particle[i].x = std::min<int>(std::max<int>(particle[i % surviving_particle_num].x + random_particle_margin(mt), 0), label_img.cols - 1);
			particle[i].y = std::min<int>(std::max<int>(particle[i % surviving_particle_num].y + random_particle_margin(mt), 0), label_img.rows - 1);
		}
		particle[i].radius = getBallRadius(label_img.rows, particle[i].x, particle[i].y);
		constexpr int minimum_radius_limit = 5;
		if(particle[i].radius < minimum_radius_limit) {
			i--;
			continue;
		}
		particle[i].score = calcParticleScore(label_img, particle[i].x, particle[i].y, particle[i].radius);
	}
	std::sort(particle.begin(), particle.end());
	constexpr double score_threshold = 0.30;
	if(particle[0].score > score_threshold) {
		object_pos pos;
		pos.type = OBJECT_BALL;
		pos.x = particle[0].x;
		pos.y = particle[0].y;
		obj.push_back(pos);
		particle_num = 100;
		random_particle_num = 30;
	} else { // detect no ball
		particle_num = 300;
		random_particle_num = 100;
	}
}

std::vector<Particle> BallParticle::getParticle(void)
{
	return particle;
}

