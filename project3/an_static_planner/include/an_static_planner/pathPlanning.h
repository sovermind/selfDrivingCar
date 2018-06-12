#ifndef PATHPLANNING_H
#define PATHPLANNING_H

// #include "ros/ros.h"
#include <vector>
#include "mapCell.h"
// #include <queue>
#include <cmath>
#include <iostream>
#include "an_static_planner/myPQ.h"
// #include "an_messages/obstacle.h"

// discrete map stored in 2D vector
std::vector<std::vector<mapCell*> > globalMap;
// store obstacles
std::vector<mapCell::obsState> allObstacles;
// motion primitives need to generate successors
std::vector<std::vector<mapCell::carState*>* > MP;
// starting epsilon for ARA*
double const epsilon_start = 3.0;
// decrease factor of the epsilon
double const decrease_factor = 0.8;
// change lane cost factor
double const change_lane_factor = 1.5;
// open list
myPQ open_list;
// incons list
myPQ incons;
// close list
std::vector<mapCell*> close_list;
// flag determine if the obstacles are updated
bool obsOK = false;
// total length 
double total_length = 0;

// construct the map
void constructGlobalMap(double total_w, double total_l, double left_Edge, double right_Edge, const double cell_size);

// process obstacles, update on global map
void processObstacles(mapCell::obsState os);

// set up a single c in global map. c can be start/goal/obstacle
// NEED MODIFY: may not need this
// void setSingleCell(mapCell* c);

// final path generate
std::vector<mapCell*> finalPathGenerate(mapCell* start_cell, mapCell* goal_cell, std::vector<std::vector<mapCell::carState*>*> allMP);

// test trajectory generator, only used when testing if planner.cpp can publish the trajectory
std::vector<mapCell*> mytestTraj();

// ARA star search algorithm
std::vector<mapCell*> ARAstar(mapCell* const start_cell, mapCell* const goal_cell, double epsilon);

// calculate heuristic value of a cell relative to the goal cell 
double heur(mapCell* cur, mapCell* goal);

// calculate the epsilon prime
double cal_eps_prime(double eps, mapCell* const goal_cell);

// get all successor nodes from cur node based on motion primitives
std::vector<mapCell*> getSucc(mapCell* cur_cell);

// calculate cost from current cell to successor cell
double cal_cost(mapCell* cur_cell, mapCell* succ);

// generate current path use back pointers from goal cell
std::vector<mapCell*> generateCurPath(mapCell* start_cell, mapCell* goal_cell);

// insert a cell pointer into the map, maintain the original cost info, but update the carState. Return the row col in gloabal map
// now only can insert car State cell, not obstacle state cell
std::vector<int> insertCellinMap(mapCell** cur_cell);

// check if certain x,y will hit obstacle. Assume car has the same dimension as obstacle
bool checkHitObs(double newx, double newy, mapCell::obsState* obs);

// give two points, and check their distance is greater than 2r
// if 2r > dist, return false
bool twoCircleHit(double x1, double y1, double x2, double y2, double r);

// take only final point of MP path, fullfill the path using MP
std::vector<mapCell*> fullFillPath(std::vector<mapCell*> fp);
#endif