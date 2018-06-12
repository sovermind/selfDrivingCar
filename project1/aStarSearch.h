#ifndef ASTARSEARCH_H
#define ASTARSEARCH_H

#include <vector>
#include "gridNode.h"
#include "myPQ.h"
#include <iostream>
#include <queue>
#include <cmath>
#include <fstream>


using namespace std;
void aStarSearch(vector<vector<gridNode> >* map, gridNode startNode, gridNode goalNode, int useAAFlag);
void calF(gridNode* curNode, gridNode goalNode);
void aaStarPrintMap(vector<vector<gridNode> >* map, gridNode* startNode, gridNode* goalNode, bool drawComplete);
vector<gridNode*> coreAstar(vector<vector<gridNode> >* map, gridNode* sn, gridNode* gn, int firstFlag, int useAAFlag, int* generatedNodeCount);
#endif