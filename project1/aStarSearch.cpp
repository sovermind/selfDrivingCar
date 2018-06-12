#include "aStarSearch.h"

struct gridNodeCompare{
	bool operator() (const gridNode* lhs, const gridNode* rhs) {
		return lhs->f > rhs->f;
	}
};

// use manhattan heuristics
void calF(gridNode* curNode, gridNode goalNode) {
	int heuristics = abs(goalNode.row - curNode->row) + abs(goalNode.col - curNode->col);
	curNode->h = heuristics;
}

// function to activate the obstacle for cur node's neighbors
void activateNeighbors(vector<vector<gridNode> >* map, gridNode* curNode) {
	// check validation of map and start and goal
	if (map == NULL) {
		cout<<"Invalid map input"<<endl;
		return;
	}
	int totalRow = map->size();
	int totalCol = map->at(0).size();
	
	// left
	if (curNode->col - 1 >= 0) {
		(&(map->at(curNode->row).at(curNode->col-1)))->isActivated=true;
	}
	// right
	if (curNode->col + 1 < totalCol) {
		(&(map->at(curNode->row).at(curNode->col+1)))->isActivated=true;
	}
	// top
	if (curNode->row - 1 >= 0) {
		(&(map->at(curNode->row-1).at(curNode->col)))->isActivated=true;
	}
	// bot
	if (curNode->row + 1 < totalRow) {
		(&(map->at(curNode->row+1).at(curNode->col)))->isActivated=true;
	}
}

// function to print out the current path
//drawComplete = true, means path found, draw complete path
// drawComplete = false, means no path found, dont draw path
void aaStarPrintMap(vector<vector<gridNode> >* map, gridNode* startNode, gridNode* goalNode, bool drawComplete) {
	ofstream outFile;
	outFile.open("output.txt", ofstream::out | ofstream::app);
	
	// check validation of map and start and goal
	if (map == NULL) {
		outFile<<"Invalid map input"<<endl;
		outFile.close();
		cout<<"Invalid map input"<<endl;
		return;
	}
	int totalRow = map->size();
	int totalCol = map->at(0).size();
	if (startNode->row >= totalRow || startNode->col >= totalCol || goalNode->row >= totalRow || goalNode->col >= totalCol) {
		outFile<<"Invalid start/goal Node"<<endl;
		outFile.close();
		cout<<"Invalid start/goal Node"<<endl;
		return;
	}

	char output[totalRow][totalCol];
	// first fill with original map info
	for (int i = 0; i < totalRow; i ++) {
		for (int j = 0; j <totalCol; j++) {
			if (map->at(i).at(j).isObstacle && map->at(i).at(j).isActivated) {
				output[i][j] = 'x';
			} else {
				output[i][j] = '_';
			}
		}
	}
	// fill in the path
	if (drawComplete) {
		gridNode* curNode  = goalNode;
		while (curNode->parent != startNode) {
			curNode = curNode->parent;
			output[curNode->row][curNode->col] = 'o';
		}
	}
	
	// fill in start and goal node, then the output is ready
	output[startNode->row][startNode->col] = 's';
	output[goalNode->row][goalNode->col] = 'g';

	// finally put the output on text file
	
	for (int ii = 0; ii < totalRow; ii ++) {
		for (int jj = 0; jj <totalCol; jj ++) {
			outFile<<output[ii][jj]<<' ';
			cout<<output[ii][jj]<<' ';
		}
		outFile<<' '<<endl;
		cout<<' '<<endl;
	}
	outFile<<"done"<<endl;
	cout<<"done"<<endl;

	outFile.close();
}

// firstFlag = 1 means the first time run the function, need to init all node's h value
// firstFlag = 0 means not the first time, don't need to init h values
// useAAFlag = 1 means use adaptive A*, which will update h
// useAAFlag = 0 means use plain A*, which wont update h
vector<gridNode*> coreAstar(vector<vector<gridNode> >* map, gridNode* sn, gridNode* gn, int firstFlag, int useAAFlag, int* generatedNodeCount, int* expandedNodeCount) {
	vector<gridNode*> thepath;
	// get the row and col from map
	int totalRow = map->size();
	int totalCol = map->at(0).size();
	// initialize close list
	vector<gridNode*> close;
	myPQ open;
	// set up boolean matrix to check if a node is in close or open
	vector<vector<bool> > isInClose(totalRow, vector<bool>(totalCol, false));
	vector<vector<bool> > isInOpen(totalRow, vector<bool>(totalCol, false));

	//calculate first time run h values
	if (firstFlag == 1) {
		for (int i = 0; i < totalRow; i++) {
			for (int j = 0; j < totalCol; j++) {
				gridNode* temp = &(map->at(i).at(j));
				calF(temp, *gn);
			}
		}
	}

	// set start node g = 0;
	sn->g = 0;
	sn->f = sn->g + sn->h;
	open.push(sn);
	isInOpen[sn->row][sn->col] = true;
	// update generated node count
	++*generatedNodeCount;

	while (!open.empty()) {
		gridNode* cur = open.top();
		open.pop();
		isInOpen[cur->row][cur->col] = false;
		// if it's the goal node, then I found the solution
		if (cur == gn) {
			// useAAFlag will determine if h been updated here
			if (useAAFlag == 1) {
				// since A* finished this turn, adaptive A* need to update h value for all nodes in close list
				for (int ii = 0; ii < close.size(); ii++) {
					close[ii]->h = gn->g - close[ii]->g;
					close[ii]->f = close[ii]->g + close[ii]->h;
				}
			}
			
			// shoule return a vector of gridnode* start from sn to gn 
			vector<gridNode*> temp;
			gridNode* curNode  = gn;
			while (curNode != sn) {
				temp.push_back(curNode);
				curNode = curNode->parent;
			}
			temp.push_back(curNode);
			for (int i = temp.size()-1; i >=0 ; i--) {
				thepath.push_back(temp[i]);
			}
			// Final check to make sure that goal node and start node in the path
			if (thepath[0] == sn && thepath[thepath.size()-1] == gn) {
				return thepath;
			} else {
				vector<gridNode*> notValidPath;
				return notValidPath;
			}
		}
		// generated node put into close list
		close.push_back(cur);
		isInClose[cur->row][cur->col] = true;
		// update generated node count and expanded Node Count
		++*generatedNodeCount;
		++*expandedNodeCount;
		// check four neighbors of current node, if out of map or is obstacle, dont add to list.
		vector<gridNode*> neighbors;
		gridNode* left;
		gridNode* right;
		gridNode* top;
		gridNode* bot;

		// Now not only need to check if the neighbour is obstacle, also need to check if it has been activated, which means that it has already been observed.
		if (cur->col - 1 >= 0) {
			left = &(map->at(cur->row).at(cur->col-1));
			if (!left->isObstacle) {
				neighbors.push_back(left);
				// cout<<"test1a"<<endl;
			} else if (left->isObstacle && !left->isActivated) {
				neighbors.push_back(left);
				// cout<<"test1b"<<endl;
			}
		}
		if (cur->col + 1 < totalCol) {
			right = &(map->at(cur->row).at(cur->col+1));
			if (!right->isObstacle) {
				neighbors.push_back(right);
				// cout<<"test2a"<<endl;
			} else if (right->isObstacle && !right->isActivated) {
				neighbors.push_back(right);
				// cout<<"test2b"<<endl;
			}
		}
		if (cur->row - 1 >= 0) {
			top = &(map->at(cur->row-1).at(cur->col));
			if (!top->isObstacle) {
				neighbors.push_back(top);
				// cout<<"test3a"<<endl;
			} else if (top->isObstacle && !top->isActivated) {
				neighbors.push_back(top);
				// cout<<"test3b"<<endl;
			}
		}
		if (cur->row + 1 < totalRow) {
			bot = &(map->at(cur->row+1).at(cur->col));
			if (!bot->isObstacle) {
				neighbors.push_back(bot);
				// cout<<"test4a"<<endl;
			} else if (bot->isObstacle && !bot->isActivated) {
				neighbors.push_back(bot);
				// cout<<"test4b"<<endl;
			}
		}
		

		for (int i = 0; i < neighbors.size(); i ++) {
			// check if the neighbor is in close set, then ignore that neighbour
			if (isInClose[neighbors[i]->row][neighbors[i]->col]) {
				continue;
			}
			// if the neighbour is not in open list either, expand that node
			if (!isInOpen[neighbors[i]->row][neighbors[i]->col]) {
				// update neighbour's g value, assume each step costs 1
				neighbors[i]->g = cur->g + 1;
				// update neighbour's f value
				neighbors[i]->f = neighbors[i]->g + neighbors[i]->h;
				// set neighbour's parent to cur
				neighbors[i]->parent = cur;
				// add the neighbour to open list
				open.push(neighbors[i]);
				isInOpen[neighbors[i]->row][neighbors[i]->col] = true;
				// update generated nodes count
				++*generatedNodeCount;
			}
			// if the neighbour is in open list
			else {
				// update neighbour's f value and parents if necessary
				// want to update the pq as well, but would be very slow
				int newg = cur->g + 1;
				int newf = newg + neighbors[i]->h;
				if (newf < neighbors[i]->f) {
					neighbors[i]->g = newg;
					neighbors[i]->f = newf;
					neighbors[i]->parent = cur;
					open.updateNode(neighbors[i]);
				}
			}
		}	
	}
	cout<<"No valid path found"<<endl;
	return thepath;
}

void aStarSearch(vector<vector<gridNode> >* map, gridNode startNode, gridNode goalNode, int useAAFlag) {
	// check validation of map and start and goal
	if (map == NULL) {
		cout<<"Invalid map input"<<endl;
		return;
	}
	int totalRow = map->size();
	int totalCol = map->at(0).size();
	if (startNode.row >= totalRow || startNode.col >= totalCol || goalNode.row >= totalRow || goalNode.col >= totalCol) {
		cout<<"Invalid start/goal Node"<<endl;
		return;
	}

	// link input startnode and goalnode with node inside map
	gridNode* sn = &(map->at(startNode.row).at(startNode.col));
	gridNode* gn = &(map->at(goalNode.row).at(goalNode.col));
	// record how many times A* run
	int count = 0;
	// rocord how many nodes have been generated
	int* generatedNodeCount;
	int* expandedNodeCount;
	int totalGeneNodeCount = 0;
	int totalExpNodeCount = 0;
	generatedNodeCount = &(totalGeneNodeCount);
	expandedNodeCount = &(totalExpNodeCount);

	// First activate the neighbor of starting cell if there's obstacles
	activateNeighbors(map, sn);
	// coreAstar will return a size > 0 vector if it successfully found a path, the last element is gauranteed to be gn
	// otherwise, it will return a size = 0 vector 
	vector<gridNode*> cur_path = coreAstar(map, sn, gn, 1, useAAFlag, generatedNodeCount, expandedNodeCount);
	// print Map
	aaStarPrintMap(map, sn, gn, true);
	count ++;
	// cout<<"done"<<endl;
	int step = 0;
	// if current step is not goal node and current step is not null(means a star find the valid solution)
	while (cur_path.size()>0 && cur_path[step] != NULL && cur_path[step] != gn) {
		// activate cur_path[step]'s neighbours if they are obstacles
		activateNeighbors(map, cur_path[step]);
		// // left
		// if (cur_path[step]->col - 1 >= 0) {
		// 	(&(map->at(cur_path[step]->row).at(cur_path[step]->col-1)))->isActivated=true;
		// }
		// // right
		// if (cur_path[step]->col + 1 < totalCol) {
		// 	(&(map->at(cur_path[step]->row).at(cur_path[step]->col+1)))->isActivated=true;
		// }
		// // top
		// if (cur_path[step]->row - 1 >= 0) {
		// 	(&(map->at(cur_path[step]->row-1).at(cur_path[step]->col)))->isActivated=true;
		// }
		// // bot
		// if (cur_path[step]->row + 1 < totalRow) {
		// 	(&(map->at(cur_path[step]->row+1).at(cur_path[step]->col)))->isActivated=true;
		// }

		if (!cur_path[step+1]->isObstacle) {
			step++;
		} else {
			// rerun a star
			// the new start node is the current node
			sn = cur_path[step];
			cur_path = coreAstar(map,sn,gn, 0, useAAFlag, generatedNodeCount, expandedNodeCount);
			count ++;
			step = 0;
			if (cur_path.size() <= 0 || cur_path[step] == NULL) {
				aaStarPrintMap(map, sn, gn, false);
			} else {
				aaStarPrintMap(map, sn, gn, true);
			} 
		}
	}

	if (cur_path.size() <= 0) {
		cout<<"No path can be found"<<endl;
	} else if (cur_path[step] == gn) {
		cout<<"navigation ended successfully"<<endl;
	} else if (cur_path[step] == NULL) {
		cout<<"No path can be found"<<endl;
	} else {
		cout<<"something unexpected happened"<<endl;
	}
	

	cout<<"Total A* runs: "<<count<<endl;
	cout<<"Total nodes been generated(close and open list): "<<totalGeneNodeCount<<endl;
	cout<<"Total nodes been expanded(close list): "<<totalExpNodeCount<<endl;
	

}









