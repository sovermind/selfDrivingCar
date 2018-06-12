#ifndef GRIDNODE_H
#define GRIDNODE_H

#include <limits>

class gridNode{
	public:
		int row;
		int col;
		int g, f, h;
		bool isObstacle;
		bool isActivated;
		
		gridNode* parent;

		gridNode(int r, int c);
		int Compare(gridNode* b);
		
};

#endif