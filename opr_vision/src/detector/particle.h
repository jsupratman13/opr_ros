#ifndef PARTICLE_H
#define PARTICLE_H

class Particle
{
public:
	Particle() : x(0), y(0), radius(0), score(0.0) {}
	Particle(const int xx, const int yy) : x(xx), y(yy), radius(0), score(0.0) {}
	Particle(const int xx, const int yy, const int r) : x(xx), y(yy), radius(r), score(0.0) {}
	Particle(const int xx, const int yy, const int r, const double s) : x(xx), y(yy), radius(r), score(s) {}
	~Particle() {}
	bool operator<(const Particle &p) const {
		return this->score > p.score;
	}
	int x;
	int y;
	int radius;
	double score;
};

#endif // PARTICLE_H

