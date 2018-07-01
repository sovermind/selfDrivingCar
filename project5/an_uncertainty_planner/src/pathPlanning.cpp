#include "an_uncertainty_planner/pathPlanning.h"

using namespace std;

void constructGlobalMap(double total_w, double total_l, double left_Edge, double right_Edge, const double cell_size) {
	int total_col = (int)(total_l/cell_size) + 1;
	int total_row = (int)(total_w/cell_size);

	for (int i = 0; i < total_row; i++) {
		vector<mapCell*> cur_row;
		for (int j = 0; j < total_col; j++) {
			// init carstate to be at the middle of each cell with theta = 0
			mapCell::carState initState;
			initState.y = left_Edge+i*cell_size + 0.5*cell_size;
			initState.x = j*cell_size + 0.5*cell_size;
			initState.theta = 0;
			initState.t = 0;

			mapCell* cur = new mapCell(initState, cell_size);
			cur->row = i;
			cur->col = j;
			cur->cs = initState;
			cur_row.push_back(cur);
		}
		globalMap.push_back(cur_row);
	}
	// cout<<"map_size: "<<globalMap.size()<<" "<<globalMap[0].size()<<endl;
	// cout<<"check111: "<<globalMap[0].back()->cs.x<<endl;
	cout<<"map construct complete"<<endl;
}

vector<mapCell*> mytestTraj() {
	vector<mapCell*> testpath;

	for (int i = 0; i < 20000; i ++) {
		mapCell* cur = new mapCell();
		mapCell::carState cs;
		cs.x = i*0.1;
		cs.y = 3.7;
		cs.theta = 0;
		cur->cs = cs;
		testpath.push_back(cur);
	}
	return testpath;
}

// NEED MODIFY: may not need this
// void setSingleCell(mapCell* c) {
// 	globalMap[c->row][c->col] = c;
// }

double heur(mapCell* cur, mapCell* goal) {
	cur->set_h(sqrt(pow(cur->row - goal->row, 2) + pow(cur->col - goal->col, 2)));
	return cur->get_h();
}

double cal_eps_prime(double eps, mapCell* const goal_cell) {
	double minoi = 1;
	if (open_list.size() == 0 && incons.size() == 0) {
		return 1;
	} else if (incons.size() == 0) {
		minoi = open_list.top()->get_f();
	} else if (open_list.size() == 0) {
		minoi = incons.top()->get_f();
	} else {
		minoi = min(open_list.top()->get_f(), incons.top()->get_f());
	}
	double eps_p = min(eps, goal_cell->get_f()/minoi);
	return eps_p;
}

vector<mapCell*> finalPathGenerate(mapCell* start_cell, mapCell* goal_cell, vector<vector<mapCell::carState*>* > allMP) {
	vector<mapCell*> fp;
	// set up the start and goal inside the global map
	vector<int> rc_s = insertCellinMap(&start_cell);
	vector<int> rc_g = insertCellinMap(&goal_cell);
	// cout<<globalMap[rc_s[0]][rc_s[1]]->row<<",s "<<globalMap[rc_s[0]][rc_s[1]]->col<<endl;
	// cout<<globalMap[rc_g[0]][rc_g[1]]->cs.x<<",g "<<globalMap[rc_g[0]][rc_g[1]]->cs.y<<endl;
	// start_cell = globalMap[rc_s[0]][rc_s[1]];
	
	// set up motion primitives
	MP = allMP;

	// initialize gv for start and goal 
	goal_cell->set_g(numeric_limits<double>::max());
	goal_cell->set_v(numeric_limits<double>::max());
	start_cell->set_v(numeric_limits<double>::max());
	start_cell->set_g(0); 
	//initialize time for start to 0
	start_cell->cs.t = 0;
	// initialize epsilon
	double epsilon = epsilon_start;
	// empty the open list
	open_list.empty();
	start_cell->set_f(epsilon*heur(start_cell, goal_cell));
	// cout<<start_cell->row<<"s "<<start_cell->col<<endl;
	// cout<<goal_cell->row<<"g "<<goal_cell->col<<endl;
	// cout<<"f: "<<start_cell->get_f()<<endl;

	open_list.push(start_cell);
	// cout<<"open: "<<open_list.top()->row<<" "<<open_list.top()->col<<endl;
	fp = ARAstar(start_cell, goal_cell, epsilon);
	double epsilon_prime = epsilon;
	while (epsilon_prime > 1) {
		epsilon = epsilon * decrease_factor;
		// cout<<"open size1: "<<open_list.size()<<endl;
		// cout<<"incons1: "<<incons.size();
		open_list.push_pq(incons);
		// cout<<"open size: "<<open_list.size()<<endl;
		for (int i = 0; i < open_list.size(); i++) {
			open_list[i]->set_f(open_list[i]->get_g() + epsilon * heur(open_list[i], goal_cell));
			open_list.updateNode(open_list[i]);
		}
		fp = ARAstar(start_cell, goal_cell, epsilon);
		epsilon_prime = cal_eps_prime(epsilon, goal_cell);
	}

	// using MP, convert end trajectory to each cell
	// cout<<"size before"<<fp.size()<<endl;
	fp = fullFillPath(fp);
	// cout<<"size after"<<fp.size()<<endl;

	// test trajectory
	// fp = mytestTraj();
	
	// test other functions
	// vector<mapCell*> succ = getSucc(start_cell);
	// for (int i = 0; i < succ.size();i++) {
	// 	double x = succ[i]->cs.x;
	// 	double y = succ[i]->cs.y;
	// 	cout<<"x: "<<x<<" y: "<<y<<endl;
	// }

	return fp;
}

vector<mapCell*> ARAstar(mapCell* const start_cell, mapCell* const goal_cell, double epsilon) {
	// vector<mapCell*> path;
	close_list.clear();
	// 2D vector keep track if a cell is in close list or not
	vector<vector<bool> > isInClose(globalMap.size(),vector<bool>(globalMap[0].size(),false));
	incons.empty();

	while (!open_list.isEmpty() && goal_cell->get_g() > open_list.top()->get_f()) {
		mapCell* cur_cell = open_list.top();
		open_list.pop();
		cur_cell->set_v(cur_cell->get_g());

		close_list.push_back(cur_cell);
		// cout<<"cur cell"<<cur_cell->row<<" "<<cur_cell->col<<endl;
		vector<mapCell*> succ_list = getSucc(cur_cell);
		// for (int i = 0; i < succ_list.size(); i++) {
		// 	cout<<i<<": "<<succ_list[i]->row<<" "<<succ_list[i]->col<<endl;
		// }
		for (int i = 0; i < succ_list.size(); i++) {
			if (!isInClose[succ_list[i]->row][succ_list[i]->col] && !open_list.isInPQ(succ_list[i])) {
				succ_list[i]->set_v(numeric_limits<double>::max());
				succ_list[i]->set_g(numeric_limits<double>::max());
			}
			// cout<<"succ i: "<<i<<endl;
			if (succ_list[i]->get_g() > cur_cell->get_g() + cal_cost(cur_cell, succ_list[i])) {
				// cout<<"g: "<<cur_cell->get_g()<<" cost"<<cal_cost(cur_cell, succ_list[i])<<endl;
				succ_list[i]->set_g(cur_cell->get_g() + cal_cost(cur_cell, succ_list[i]));
				if (!isInClose[succ_list[i]->row][succ_list[i]->col]) {
					succ_list[i]->set_f(succ_list[i]->get_g() + epsilon * heur(succ_list[i], goal_cell));
					succ_list[i]->parent = cur_cell;
					if (!open_list.isInPQ(succ_list[i])) {
						open_list.push(succ_list[i]);
					} else {
						open_list.updateNode(succ_list[i]);
					}
				} else {
					if (!incons.isInPQ(succ_list[i])) {
						incons.push(succ_list[i]);
					}
				}
			}
		}
	}
	return generateCurPath(start_cell, goal_cell);
	// vector<mapCell*> c;
	// return c;
}

vector<mapCell*> getSucc(mapCell* cur_cell) {
	vector<mapCell*> succ_list;
	mapCell::carState cur_state = cur_cell->cs;
	// cout<<"cur state"<<cur_state.x<<" "<<cur_state.y<<endl;

	// make sure MP and obstacles are setup
	if (MP.size()<=0 || !obsOK) {
		cout<<"WARNING: Motion Primitives not setup"<<endl;
		return succ_list;
	}
	// cout<<"obstacle size: "<<allObsTime.size()<<endl;
	// for (int ii = 0; ii < allObsTime.at(0).size(); ii++) {
	// 	cout<<ii<<" x: "<<allObsTime.at(0).at(ii).x<<" y: "<<allObsTime.at(0).at(ii).y<<endl;
	// }
	// for (auto it = allObsTime.cbegin();it != allObsTime.cend(); ++it) {
	// 	cout<<it->first<<endl;
	// }
	// cout<<MP[0]->size()<<endl;
	// for each MP, develop successors
	for (int i =0; i < MP.size(); i++) {
		// check if within left boundary
		mapCell::carState endState = *(MP[i]->back());
		if (endState.y + cur_state.y < globalMap[0][0]->cs.y) {
			continue;
		}
		// check if within right boundary
		if (endState.y + cur_state.y > globalMap[globalMap.size()-1][0]->cs.y) {
			continue;
		}
		// check if outside the map
		if (endState.x + cur_state.x > globalMap[0].back()->cs.x) {
			continue;
		}
		// check if hit obstacle
		bool hit = false;
		for (int j = 0; j < MP[i]->size(); j++) {
			// cout<<"check ith mp, jth point"<<i<<" "<<j<<endl;
			double new_x = cur_state.x + MP[i]->at(j)->x;
			double new_y = cur_state.y + MP[i]->at(j)->y;
			double new_t = cur_state.t + MP[i]->at(j)->t;
			// cout<<"check xy " <<new_x<<" "<<new_y<<endl;

			// determine the closest time value
			double remain = fmod(new_t, obsTimeResolution);
			if (remain == obsTimeResolution) {
				// right at that time
			} else if (remain <= 0.025) {
				new_t = new_t - remain;
			} else {
				new_t = new_t - remain + obsTimeResolution;
			}
			new_t = round(100*new_t)/100;
			// cout<<"t: "<<new_t<<" remain: "<<remain<<endl;
			vector<mapCell::obsState> allObstacles;

			// cout<<"largest "<<allObsTime.rbegin()->first<<endl;
			// make sure the time is within obstacles' largest time
			if (new_t >= allObsTime.rbegin()->first) {
				new_t = allObsTime.rbegin()->first;
			}
			// if (allObsTime.find(new_t) == allObsTime.end()) {
			// 	cout<<"not find: "<<new_t<<endl;
			// }
			allObstacles = allObsTime.at(new_t);

			for (int k = 0; k < allObstacles.size(); k++) {
				// cout<<"check ith mp, jth point, kth obs"<<i<<" "<<j<<" "<<k<<endl;
				// cout<<"hit obs: "<<allObstacles[k]->x<<" "<<allObstacles[k]->y<<endl;
				if (checkHitObs(new_x, new_y, &allObstacles[k])) {
					hit = true;
					// cout<<"hit obs: " <<new_x<<" "<<new_y<<" "<<allObstacles[k].x<<" "<<allObstacles[k].y<<endl;
					break;
				}
			}
			if (hit) {
				break;
			}
		}
		if (!hit) {
			mapCell* succ = new mapCell();
			succ->cs = endState;
			succ->cs.x = succ->cs.x + cur_state.x;
			succ->cs.y = succ->cs.y + cur_state.y;
			succ->cs.t = succ->cs.t + cur_state.t;
			// insert into map, and will also make succ_s point to cell in global map
			insertCellinMap(&succ);
			succ_list.push_back(succ);
		}
	}

	return succ_list;

}

double cal_cost(mapCell* cur_cell, mapCell* succ) {
	if (succ->row == cur_cell->row) {
		return succ->col - cur_cell->col;
	} else {
		return change_lane_factor*(succ->col - cur_cell->col);
	}
}

// when generating path, fill in all cells between path with corresponding car states
vector<mapCell*> generateCurPath(mapCell* start_cell, mapCell* goal_cell) {
	vector<mapCell*> path;
	//path.push_back(goal_cell);
	mapCell* c = goal_cell;
	// cout<<"cp: "<<goal_cell->parent->row<<" "<<goal_cell->parent->col<<endl;
	while (c->parent != start_cell) {
		path.push_back(c);
		c = c->parent;
	}
	// cout<<"check point"<<endl;
	path.push_back(c);
	path.push_back(c->parent);

	for (int i = 0; i < path.size()/2; i++) {
		// swap(path, i, path.size()-1-i);
		mapCell* temp = path[i];
		path[i] = path[path.size()-1-i];
		path[path.size()-1-i] = temp;
	}
	return path;
}

// for this project, I will need to add time vetor to each obstacles
void processObstacles(mapCell::obsState os, double cur_time) {
	// store all obstacles
	// for (int ii = 0; ii < allObstacles.size(); ii++) {
	// 	cout<<"obs a: "<<allObstacles[ii]->x<<" "<<allObstacles[ii]->y<<endl;
	// }
	// cout<<"start this cycle"<<endl;

	// allObstacles.push_back(os);
	
	// cout<<"obs: "<<os->x<<" "<<os->y<<endl;
	// for (int ii = 0; ii < allObstacles.size(); ii++) {
	// 	cout<<"obs b: "<<allObstacles[ii].x<<" "<<allObstacles[ii].y<<endl;
	// }
	// cout<<"finish this cycle"<<endl;
	cur_time = round(cur_time*100)/100;
	if (allObsTime.find(cur_time) == allObsTime.end()) {
		// if time has not been inserted
		vector<mapCell::obsState> thisTimeObs;
		thisTimeObs.push_back(os);
		allObsTime.insert(pair<double, vector<mapCell::obsState> >(cur_time, thisTimeObs));
	} else {
		// if time has been inserted
		allObsTime.at(cur_time).push_back(os);
	}
	obsOK = true;
}

// now only can insert car State cell, not obstacle state cell
vector<int> insertCellinMap(mapCell** cur_cell) {
	vector<int> rc;
	if (globalMap.size() <= 0 || globalMap[0].size() <= 0) {
		return rc;
	}
	// double cell_size = (*cur_cell)->get_cell_size();
	double cell_size = globalMap[0][0]->get_cell_size();
	// mapCell::carState largestCell = globalMap[globalMap.size()-1][globalMap[0].size()-1]->cs;
	// 0,0 position in map
	mapCell::carState* ss = &(globalMap[0][0]->cs);
	mapCell::carState cur_s = (*cur_cell)->cs;
	int x_offset = (int)((cur_s.x - ss->x) / cell_size);
	int y_offset = (int)((cur_s.y - ss->y) / cell_size);

	// cout<<"icm"<<y_offset<<" "<<x_offset<<endl;
	// cout<<"icm"<<globalMap.size()<<" "<<globalMap[0].size()<<endl;
	// cur_cell->row = x_offset;
	// cur_cell->col = y_offset;
	if (y_offset > globalMap.size() || x_offset > globalMap[0].size()+1) {
		return rc;
	}
	// set the corresponding cell's carState equal to current carState
	globalMap[y_offset][x_offset]->cs = cur_s;
	// make sure that pointer get in also point to the correct map Cell
	// this way will make sure all other field in cell are correctly updated
	// x_offset is col and y_offset is col
	(*cur_cell) = globalMap[y_offset][x_offset];
	// cout<<(*cur_cell)->row<<"ism "<<(*cur_cell)->col<<endl;
	rc.push_back(y_offset);
	rc.push_back(x_offset);
	return rc;
}

bool checkHitObs(double newx, double newy, mapCell::obsState* obs) {
	double dist = sqrt(pow(newx - obs->x, 2) + pow(newy - obs->y, 2));
	// cout<<"check hit obs"<<dist<<endl;
	// check outer radius and inner radius
	if (dist > 2*obs->outerR) {
		return false;
	}
	if (dist < 2*obs->innerR) {
		return true;
	}
	// then check three circles that encircle the car&obstacles
	double r = obs->threeCircleR;
	// cout<<"inner r: "<<obs->innerR<<endl;
	// cout<<"3 circle r: "<<r<<endl;
	if (twoCircleHit(newx - obs->x_offset, newy, obs->x - obs->x_offset, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx, newy, obs->x - obs->x_offset, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx + obs->x_offset, newy, obs->x - obs->x_offset, obs->y, r)) {
		return true;
	}

	if (twoCircleHit(newx - obs->x_offset, newy, obs->x, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx, newy, obs->x, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx + obs->x_offset, newy, obs->x, obs->y, r)) {
		return true;
	}

	if (twoCircleHit(newx - obs->x_offset, newy, obs->x + obs->x_offset, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx, newy, obs->x + obs->x_offset, obs->y, r)) {
		return true;
	}
	if (twoCircleHit(newx + obs->x_offset, newy, obs->x + obs->x_offset, obs->y, r)) {
		return true;
	}

	return false;
}

bool twoCircleHit(double x1, double y1, double x2, double y2, double r) {
	double dist = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	double buffer_offset = 0.5;
	if (dist < 2 * r + buffer_offset) {
		return true;
	} 
	return false;
}

vector<mapCell*> fullFillPath(vector<mapCell*> fp) {
	vector<mapCell*> full_fp;
	double dist = 0;

	for (int i = 0; i < fp.size()-1; i++) {
		// insert the current point
		// full_fp.push_back(fp[i]);
		// check the next point
		for (int j = 0; j < MP.size(); j++) {
			// if it's this MP
			if (fp[i]->cs.x + MP[j]->back()->x == fp[i+1]->cs.x && fp[i]->cs.y + MP[j]->back()->y == fp[i+1]->cs.y) {
				mapCell::carState prev_cs;
				for (int k = 0; k < MP[j]->size()-1; k++) {
					mapCell* cur = new mapCell();
					mapCell::carState cs;
					cs.x = fp[i]->cs.x + MP[j]->at(k)->x;
					cs.y = fp[i]->cs.y + MP[j]->at(k)->y;
					cs.theta = MP[j]->at(k)->theta;
					cs.t = fp[i]->cs.t + MP[j]->at(k)->t;
					cur->cs = cs;
					if (k > 0) {
						double cur_dis = sqrt(pow(cs.x - prev_cs.x, 2) + pow(cs.y - prev_cs.y, 2));
						dist = dist + cur_dis;
					}
					prev_cs = cs;
					full_fp.push_back(cur);
				}
				total_length = dist;
				break;
			}
		}
		
	}
	full_fp.push_back(fp.back());
	// cout<<"size"<<full_fp.size()<<endl;

	return full_fp;
}