# Project 1 -- A* & adaptive A*
## By Jiarui Yang

### Summary of the project:
This project implemented A* and Adaptive A* searching algorithm. The whole project will take a text file as input. The input file should contain the map information and formatted as follows:
```
Number of row in gridworld
Number of col in gridworld
gridworld map
```
The gridworld map should be represented as follows:
```
s: start cell
g: goal cell
x: obstacle
_: unblocked cell
```
After running the project, there should be output text file being generated and containing several maps. Assuming no information over the whole map at very first, as travel from start cell, each time it sees an obstacle(appearing at left, right, up and down cells), it will update that information in the map. Therefore, it will first run a A*/Adaptive A* search over a blank map, then travel along the path. Everytime there's an obstacle on the path, it will re-run the search algorithm to re-calculate the path, and follow the new path. Thus each time it re-plan the path, output file will record it's current path and information over the map.

To run the project, first make sure g++ compiler is installed. Then using the [makefile](https://gitlab.com/sovermind/student1707024/blob/master/project1/makefile). In command window, type `make`. The object and executable files will be generated. To clear the folder, type `make clean`.

Then use `./JerryPro1 -A -i input.txt` to run the executable file.

`-A` or `-a` both works to trigger the adaptive A* algorithm. If not putting this command, it will just run A*.

`-i` will follow by the input file name with the .txt extension.

Make sure that all files are under the same folder directory. The `output.txt` will also be generated under the same directory.

And a detailed report is also submitted.

### Summary of each file:
#### 1. [main.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project1/main.cpp)
This is the main file containing main function. It handles the file input and command line input. It will decide whether to call A* or adaptive A* depending on the command line input.
#### 2. [aStarSearch.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project1/aStarSearch.cpp) & [aStarSearch.h](https://gitlab.com/sovermind/student1707024/blob/master/project1/aStarSearch.h):
These are the main logic file containing A* and adaptive A* search. Assuming no information of the map, as walking inside the map, gain information of blocked cells of neighbors.
#### 3. [gridNode.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project1/gridNode.cpp) & [gridNode.h](https://gitlab.com/sovermind/student1707024/blob/master/project1/gridNode.h)
gridNode class contains information of one grid node, including row, col, parent, is obstacle, is activated(if the gird is seen by the moving node). Also the gridNode will have g, h, f cost and a compare function which compares the f value.
#### 4. [myPQ.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project1/myPQ.cpp) & [myPQ.h](https://gitlab.com/sovermind/student1707024/blob/master/project1/myPQ.h)
customized class of priority queue for gridNodes. This only works for gridNode and will support pop, top, push functions. And also will support update function which can maintain the PQ property after a certain gridNode value changes. STL priority queue don't have this functionality.
