#include "ros/ros.h"
#include <vector>
#include <cstdio>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "an_static_planner/mapCell.h"
#include "tf/tf.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include "an_messages/obstacles.h"
#include "an_messages/obstacle.h"
#include <limits>
#include <cmath>
#include "an_static_planner/pathPlanning.h"
// #include "tf/transform_broadcaster.h"

class Planner_c {
public:
    ros::NodeHandle nh_;
    
    // Initialization function: init all variables and take in Motion primitives from rosparam
    bool Init_(void);
    // loop function to do the planning
    void Loop_(void);
    // take in motion primitives and store them locally
    void loadMP();
    // all call back functions to take necessary information
    void goalsub_callback(const geometry_msgs::PoseStamped& msg);
    void posesub_callback(const geometry_msgs::PoseStamped& msg);
    void mapsub_callback(const an_messages::lanes& msg);
    void obstacle_callback(const an_messages::obstacles& msg);
    // plan function to be called once and publish the path to planner_trajectory topic
    // add a duration to the cycle, so that when rviz will recieve the obstacles before moving the ego vehicle
    an_messages::trajectory plan(ros::Duration d);

private:
	// all flags for getting varaible messages
	bool new_plan_ = true;
	bool pose_recieved_ = false;
	bool goal_recieved_ = false;
	bool obstacle_recieved_ = false;
	bool map_recieved_ = false;
	// set cell size for the map
	const double cell_size = 0.5;
	// constant velocity of vehicle
	const double car_velocity = 25;
	// goal info
	mapCell* goal_cell;
	// initial start point
	mapCell* start_cell;
	// Motion primitives, containing all single MP. The order is straight, left, right
	std::vector<std::vector<mapCell::carState*>*> allMP;
	// single motion primitives
	std::vector<mapCell::carState*> straightMP;
	std::vector<mapCell::carState*> leftMP;
	std::vector<mapCell::carState*> rightMP;

	// function to load one single PM. Take the open file fp and MP name as argument
	void loadSingleMP(FILE* fp, char mp_name[]);	
};
