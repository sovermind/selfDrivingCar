#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>
#include "gridNode.h"
#include "aStarSearch.h"

using namespace std;
// global variables
bool useAAStar = false;                     // determine if use aa star
vector<vector<gridNode> > theMap;          // global map
gridNode startNode(-1,-1);                 // start node
gridNode goalNode(-1,-1);                  // goal node

// FUNCTIONS:
// given file name to read in input file and get a 2d matrix as map
bool fileReader(string fName);

int main(int argc, char* argv[]) 
{
	// Initialize input file name
	string inputFileName = "No input file";
	// Handle the input argument
	for (int i = 0; i < argc; i++) {
		// get the input file
		if (strcmp(argv[i],"-i")==0 && i+1 < argc) {
			inputFileName = argv[i+1];
		} 
		if (strcmp(argv[i],"-A")==0 || strcmp(argv[i],"-a")==0 ) {
			useAAStar = true;
		}
	}

	// Use input file to generate the map, set up the start and goal nodes
	if (fileReader(inputFileName)) {
		cout<<"Map construct complete"<<endl;
		if (useAAStar) {
			// adaptive a star search
			aStarSearch(&theMap, startNode, goalNode, 1);
		} else {
			// a star search
			aStarSearch(&theMap, startNode, goalNode, 0);
		}
	} else {
		cout<<"Map construct failed"<<endl;
	}

    return 0;
}

// read the file and filled in the 2D vector Map
bool fileReader(string fName) {
	ifstream inputFile(fName);
	string line;
	if (inputFile.is_open()) {
		int count = 0;
		int row = 0;
		int col = 0;
		while (getline(inputFile, line)) {
			// first line is row numb
			if (count == 0) {
				row = stoi(line);
				//cout<<row<<endl;
			}
			// second line is col numb 
			else if (count == 1) {
				col = stoi(line);
				theMap.reserve(row);
				//cout<<col<<endl;
			}
			// the rest is info for each col
			else if (count >= 2) {
				// seperate the line into each cells
				istringstream iss(line);
				string curCell = "null";
				// current row's temp vector for grid nodes
				vector<gridNode> tempVector;
				tempVector.reserve(col);

				for (int i = 0; i < col; i++) {
					iss >> curCell;
					// cout<<curCell<<endl;
					// save each node into temp vector
					gridNode tempNode(count-2, i);
					// check if is start or goal node
					if (curCell == "g") {
						goalNode = tempNode;
					} else if (curCell == "s") {
						startNode = tempNode;
					} else if (curCell == "x") {
						// check if it's an obstacle
						tempNode.isObstacle = true;
					}
					tempVector.push_back(tempNode);
				}
				theMap.push_back(tempVector);
				
			}			
			count++;
			
		}
		inputFile.close();
		return true;
	}
	return false;
}








