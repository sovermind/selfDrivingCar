#include "an_dynamic_planner/mapCell.h"

using namespace std;

mapCell::mapCell() {
	g = numeric_limits<double>::max();
	f = numeric_limits<double>::max();
	v = numeric_limits<double>::max();
	isObstacle = false;
}

mapCell::mapCell(carState cs, const double cell_size) {
	this->cs = cs;
	this->cell_size = cell_size;

	g = numeric_limits<double>::max();
	f = numeric_limits<double>::max();
	v = numeric_limits<double>::max();
	isObstacle = false;
}

mapCell::mapCell(obsState os, const double cell_size) {
	this->os = os;
	this->cell_size = cell_size;
	isObstacle = true;
}

void mapCell::set_g(double a) {g = a;}
void mapCell::set_f(double a) {this->f = a;}
void mapCell::set_h(double a) {this->h = a;}
void mapCell::set_v(double a) {this->v = a;}


double mapCell::get_g() {return g;}
double mapCell::get_f() {return f;}
double mapCell::get_h() {return h;}
double mapCell::get_v() {return v;}

double mapCell::get_cell_size() {return cell_size;}

int mapCell::Compare(mapCell* b) {
	if (f < b->f) {
		return -1;
	} else if (f == b->f) {
		// f values are the same, use g value to determine
		// experimentally test if small-favored (g< b->g)/large-favored
		if (g < b->g) {
			return -1;
		} else if (g > b->g) {
			return 1;
		} else {
			return 0;
		}
		return 0;
	} else {
		return 1;
	}
}