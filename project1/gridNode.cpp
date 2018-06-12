#include "gridNode.h"

gridNode::gridNode(int r, int c) {
	row = r;
	col = c;
	g = std::numeric_limits<int>::max();
	f = std::numeric_limits<int>::max();
	h = 0;
	isObstacle = false;
	isActivated = false;
}

int gridNode::Compare(gridNode* b) {
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