#ifndef MYPQ_H
#define MYPQ_H

#include "gridNode.h"
#include <vector>
#include <iostream>

using namespace std;

// the priority queue only for gridNode class
class myPQ{
	private:
		vector<gridNode*> pq;
		int pq_realSize;
	public:
		//constructor
		myPQ();
		int parent(int i);
		gridNode* top();
		void pop();
		void push(gridNode* node);
		void updateNode(gridNode* node);
		void heapify(int cur_idx);
		bool empty();
};

#endif