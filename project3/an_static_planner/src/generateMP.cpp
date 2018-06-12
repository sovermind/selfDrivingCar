#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <math.h>
#include <graphics.h>

using namespace std;

int const vel = 25; // m/s
// http://toledo.net.technion.ac.il/files/2012/12/TRR_ToledoZohar_07.pdf
// the paper state that the mean lane change duration on a high way is around 3.5s
double const totalTime = 4; // s
// total length of straight MP is totaltime * velocity
// and this will be the total x length for curved MP
double const totalLength = totalTime * vel;
double const laneWidth = 3.7;

// struct for different states, should contain x, y, theta, t
struct carState {
	double x; // along the lane
	double y; // perpendicular to the lane
	double theta;
	double t;
};

// given a vector of MP and the name of that MP, write to output file
void writeToFile(vector<carState*> theMP, string name) {
	ofstream outFile;
	outFile.open("mprim.txt", ofstream::out | ofstream::app);
	//put down the name and the number of MP here
	outFile<<name<<" "<<theMP.size()<<endl;


	for (int i = 0; i < theMP.size();i++) {
		double cur_x = theMP[i]->x;
		double cur_y = theMP[i]->y;
		double cur_delta = theMP[i]->theta;
		double cur_time = theMP[i]->t;
		outFile<<cur_x<<" "<<cur_y<<" "<<cur_delta<<" "<<cur_time<<endl;
	}
	
}

// given an instruction, then return a vector of carState based on dubins curve
vector<carState*> dubinsCurve(bool isUpCurve) {
    // for graph purpose, the coefficient
    double coef = 2.0;

	vector<carState*> dbC;

	// total curve angle in rad
	double total_curve_angle = 0.04;
	// curvature = 1/R <=0.1 & lateral acceleration = v^2/R < 3 m/s^2
	double curve_Radius = 210;
	// one single step MP length
	double deltaL = 0.8;
	double delta_theta = deltaL/curve_Radius;
	int curve_count = total_curve_angle/delta_theta + 1;
	double prev_x = 0;
	double prev_y = 0;
	double ending_theta;
	//for the first curve part
	for (int i = 0; i <curve_count; i++ ) {
		carState* cur = new carState();
		cur->x = curve_Radius*sin(i*delta_theta);
		cur->y = curve_Radius - curve_Radius*cos(i*delta_theta);
		cur->theta = i*delta_theta;
		if (isUpCurve) {
			cur->y = (cur->y)*(-1);
			cur->theta = -i*delta_theta;
		}
		cout<<cur->y<<endl;
		// cur->theta = i*delta_theta;
		ending_theta = cur->theta;
		// cout<<"x: "<<cur->x<<"y: "<<cur->y<<"theta: "<<cur->theta<<endl;

		// line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);

		// if (!isUpCurve) {
		// 	line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);
		// } else {
		// 	line(coef*prev_x+80,400-coef*prev_y,coef*cur->x+80,400-coef*cur->y);
		// }		
		prev_x = cur->x;
		prev_y = cur->y;
		dbC.push_back(cur);
	}
	// // calculate the x length of the straight part
	// double straight_xL = totalLength - 2*prev_x;
	// cout<<"xl "<<prev_x<<endl;
	// cout<<"sl "<<straight_xL<<endl;
	// if (straight_xL <=0) {
	// 	cout<<"wrong choice of curvature"<<endl;
	// 	return dbC;
	// }
	//calculate the y length of the straight part
	double straight_yL = 0;
	if (isUpCurve) {
		straight_yL = 3.7 + 2*prev_y;
	} else {
		straight_yL = 3.7 - 2*prev_y;
	}
	cout<<"yl "<<prev_y<<endl;
	cout<<"sl "<<straight_yL<<endl;
	if (straight_yL <=0) {
		cout<<"wrong choice of curvature"<<endl;
		return dbC;
	}
	double ending_x = prev_x;
	double ending_y = prev_y;
	cout<<"y:"<<ending_y<<endl;
	// for the straight line part
	// int straight_count = (straight_xL/cos(total_curve_angle)) / deltaL;
	int straight_count = (straight_yL/sin(total_curve_angle)) / deltaL;
	for (int j = 0; j < straight_count;j++) {
		carState* cur = new carState();
		cur->x = ending_x + j*deltaL*cos(total_curve_angle);
		
		if (isUpCurve) {
			cur->y = ending_y - j*deltaL*sin(total_curve_angle);
		} else {
			cur->y = ending_y + j*deltaL*sin(total_curve_angle);
		}
		// cur->theta = total_curve_angle;
		// ending_theta = cur->theta;
		cur->theta = ending_theta;
		// cout<<"x: "<<cur->x<<"y: "<<cur->y<<"theta: "<<cur->theta<<endl;

		// line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);

		// if (!isUpCurve) {
		// 	line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);
		// } else {
		// 	line(coef*prev_x+80,400-coef*prev_y,coef*cur->x+80,400-coef*cur->y);
		// }
		prev_x = cur->x;
		prev_y = cur->y;
		dbC.push_back(cur);
	}

	ending_x = prev_x;
	ending_y = prev_y;
	// for the last curve part
	for (int k = 0; k <= curve_count;k++) {
		carState* cur = new carState();
		cur->x = ending_x + curve_Radius * sin(total_curve_angle) - curve_Radius*sin(total_curve_angle - k*delta_theta);
		if (isUpCurve) {
			cur->y = ending_y - curve_Radius * cos(total_curve_angle - k*delta_theta) + curve_Radius * cos(total_curve_angle);
			cur->theta = -(total_curve_angle - k*delta_theta);
		} else {
			cur->y = ending_y + curve_Radius * cos(total_curve_angle - k*delta_theta) - curve_Radius * cos(total_curve_angle);
			cur->theta = total_curve_angle - k*delta_theta;
		}
		// cur->theta = total_curve_angle - k*delta_theta;
		ending_theta = cur->theta;
		//cout<<"x: "<<cur->x<<"y: "<<cur->y<<"theta: "<<cur->theta<<endl;

		// line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);

		// if (!isUpCurve) {
		// 	line(coef*prev_x+80,400+coef*prev_y,coef*cur->x+80,400+coef*cur->y);
		// } else {
		// 	line(coef*prev_x+80,400-coef*prev_y,coef*cur->x+80,400-coef*cur->y);
		// }		
		prev_x = cur->x;
		prev_y = cur->y;
		dbC.push_back(cur);
	}
	//final check of final state
	carState* finalState = new carState();
	finalState->x = totalLength;
	finalState->y = laneWidth;
	if (isUpCurve) {
		finalState->y = -laneWidth;
	}
	finalState->theta = 0;
	dbC.push_back(finalState);
	// cout<<"x: "<<dbC[dbC.size()-1]->x<<"y: "<<dbC[dbC.size()-1]->y<<"theta: "<<dbC[dbC.size()-1]->theta<<endl;
	return dbC;
}

void generateStraight() {
	vector<carState*> straightMP;
	// total count is number of intermediate points
	int totalCount = 1.4*totalLength;
	for (int i = 0; i<= totalCount; i++) {
		carState* cur = new carState();
		cur->x = i*totalLength/totalCount;
		cur->y = 0.0;
		cur->theta = 0.0;
		cur->t = i*totalTime/totalCount;
		// cout<<"x: "<<cur->x<<",t: "<<cur->t<<"i "<<i<<endl;
		straightMP.push_back(cur);
	}
	writeToFile(straightMP, "straight");
}

void generateLeftTurn() {
	vector<carState*> leftTurn = dubinsCurve(true);
	double delta_time = totalTime/(leftTurn.size()-1);
	//set up all the time variable
	for (int i = 0; i < leftTurn.size();i++) {
		leftTurn[i]->t = i*delta_time;
	}
	writeToFile(leftTurn,"leftTurn");
}

void generateRightTurn() {
	vector<carState*> rightTurn = dubinsCurve(false);
	double delta_time = totalTime/(rightTurn.size()-1);
	//set up all the time variable
	for (int i = 0; i < rightTurn.size();i++) {
		rightTurn[i]->t = i*delta_time;
	}
	writeToFile(rightTurn,"rightTurn");
}

int main() {
	// int gd = DETECT, gm;
	// initgraph initializes the graphics system
    // by loading a graphics driver from disk
    // initgraph(&gd, &gm, NULL);
    generateStraight();
    generateLeftTurn();
    generateRightTurn();
	// getch();
	
	// closegraph function closes the graphics
    // mode and deallocates all memory allocated
    // by graphics system .
    // closegraph();
	// generateStraight();
	

	//cout<<"vel is "<<vel<<endl;
	return 0;
}