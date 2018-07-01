#ifndef MAPCELL_H
#define MAPCELL_H 

#include <vector>
#include <cstdio>
#include <limits>

// map cell class represent each cell in the map
class mapCell {
private:
	// the edge length of the cell
	double cell_size;
	// g, f, h value for path planning
	double g, f, h, v;

	bool isObstacle;
	bool isActivated;
	
	//int Compare(mapCell* b);

public:
	// car state struct
	struct carState {
		double x;
		double y;
		double theta;
		double t;
	};
	// obstacle state struct
	struct obsState {
		double x;
		double y;
		double innerR;
		double outerR;
		double threeCircleR;
		double x_offset;
	};

	carState cs;
	obsState os;
	// constructor take car state with true position x, y, and the cell size
	// then it should calculate the row & col of this cell
	mapCell();
	mapCell(carState cs, const double cell_size);
	// another constructor take obstacle state with true position x, y, and the cell size
	mapCell(obsState os, const double cell_size);
	// row, col represent the position of this cell inside the whole cell map
	int row;
	int col;
	// setter for g f h v
	void set_g(double a);
	void set_f(double a);
	void set_h(double a);
	void set_v(double a);

	// getter for g f h v
	double get_g();
	double get_f();
	double get_h();
	double get_v();

	// get cell size
	double get_cell_size();
	// compare function, compare f value, break tie upon g value
	int Compare(mapCell* b);
	// parent cell
	mapCell* parent;


};

#endif