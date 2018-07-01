#ifndef MYPQ_H
#define MYPQ_H

#include "mapCell.h"
#include <vector>
#include <iostream>

// the priority queue only for mapCell class
// copy and modified from project 1
class myPQ {
	private:
		std::vector<mapCell*> pq;
		int pq_realSize;
	public:
		//constructor
		myPQ();
		int parent(int i);
		// return the first element in pq
		mapCell* top();
		// remove the first element
		void pop();
		// push one element in, and maintain pq properties
		void push(mapCell* node);
		// after a node's value change, update the pq so that the node can be moved to right position
		void updateNode(mapCell* node);
		// heapify the pq start from cur_idx, so that the pq maintain its properties
		void heapify(int cur_idx);
		// determine if the pq is empty
		bool isEmpty();
		// empty the whole pq
		void empty();
		// push another pq into this, only save the same elements
		void push_pq(myPQ another);
		// determine if a node is in pq
		bool isInPQ(mapCell* node);
		int size();
		mapCell* operator [] (int i);
};

#endif