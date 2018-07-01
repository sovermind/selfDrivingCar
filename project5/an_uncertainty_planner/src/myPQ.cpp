#include "an_uncertainty_planner/myPQ.h"

using namespace std;

// constructor
myPQ::myPQ() {
	myPQ::pq_realSize = 0;
}

int myPQ::parent(int i) {
	return (i-1)/2;
}

// get the top element
mapCell* myPQ::top() {
	if (myPQ::pq_realSize> 0) {
		return myPQ::pq[0];
	} else {
		return NULL;
	}
}

// check if the pq is empty
bool myPQ::isEmpty() {
	if (myPQ::pq_realSize <= 0) {
		return true;
	} else {
		return false;
	}
}

// get rid of the first element of pq
void myPQ::pop() {
	if (pq_realSize <= 0) {
		return;
	}
	if (pq_realSize == 1) {
		pq_realSize --;
		myPQ::pq[0] = NULL;
		return;
	}
	myPQ::pq[0] = myPQ::pq[pq_realSize-1];
	pq_realSize--;
	myPQ::heapify(0);
}

// add a node to the pq
void myPQ::push(mapCell* node) {
	// first put the node at the end of the heap
	if (myPQ::pq.size() > myPQ::pq_realSize) {
		myPQ::pq_realSize++;
		myPQ::pq[pq_realSize-1] = node;
	} else {
		myPQ::pq_realSize++;
		myPQ::pq.push_back(node);
	}
	int i = pq_realSize-1;
	// bubble the new node up to the correct position
	while (i != 0 && pq[parent(i)] -> Compare(pq[i]) == 1) {
		swap(pq[parent(i)], pq[i]);
		i = parent(i);
	}

}

// update the pq when input node's f value change
void myPQ::updateNode(mapCell* node) {
	for (int i = 0; i < pq_realSize; i++) {
		if (pq[i] == node) {
			while (i != 0 && pq[parent(i)] -> Compare(pq[i]) == 1) {
				swap(pq[parent(i)], pq[i]);
				i = parent(i);
			}
			return;
		}
	}
	
}

// heapify the pq start from cur_idx, so that the pq maintain its properties
void myPQ::heapify(int cur_idx) {
	int left = 2*cur_idx + 1;
	int right = 2*cur_idx + 2;
	int smallest = cur_idx;
	if (left < pq_realSize && pq[left]->Compare(pq[cur_idx]) == -1) {
		smallest = left;
	}
	if (right < pq_realSize && pq[right]->Compare(pq[smallest]) == -1) {
		smallest = right;
	}
	if (smallest != cur_idx) {
		swap(pq[cur_idx], pq[smallest]);
		myPQ::heapify(smallest);
	}
}

void myPQ::empty() {
	while (!isEmpty()) {
		pop();
	}
}

void myPQ::push_pq(myPQ another) {
	if (another.size() <= 0) {
		return;
	}
	if (!isInPQ(another.top())) {
		push(another.top());
	}
	another.pop();
}

bool myPQ::isInPQ(mapCell* node) {
	for (int i = 0; i < pq_realSize; i++) {
		if (pq[i] == node) {
			return true;
		}
	}
	return false;
}

int myPQ::size() {
	return pq_realSize;
}

mapCell* myPQ::operator [] (int i) {
	return pq[i];
}