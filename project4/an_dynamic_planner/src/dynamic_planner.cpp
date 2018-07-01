#include "an_dynamic_planner/dynamic_planner.h"

using namespace std;

an_messages::trajectory Planner_c::plan(ros::Duration d) {
	// call final Path Generate function from pathPlanning class
	vector<mapCell*> final_path = finalPathGenerate(start_cell, goal_cell, allMP);
	// the traj msg that can be returned
	an_messages::trajectory completeTraj_msg;
	if (final_path.size()<=0) {
		ROS_WARN("[planner] Error when generating path");
		return completeTraj_msg;
	}
	// convert vector of mapCell to trajectory messages
	int traj_size = final_path.size();
	completeTraj_msg.traj.resize(traj_size);

	ros::Time start = ros::Time::now();
	double prev_x = final_path[0]->cs.x;
	double prev_y = final_path[0]->cs.y;
	for (int i = 0; i < traj_size; i++) {
		mapCell* cur = final_path[i];
		mapCell::carState cs = cur->cs;
		an_messages::traj_pt cur_pt;
		cur_pt.position.x = cs.x;
		cur_pt.position.y = cs.y;
		cur_pt.position.theta = cs.theta;

		// set up the time stamp and frame id
		double dist_diff = sqrt(pow(cs.x - prev_x, 2) + pow(cs.y - prev_y, 2));
		// ros::Duration d(5);
		// cur_pt.header.stamp = start + d + ros::Duration(dist_diff/car_velocity);
		// TODO: Need to sync with the obstacles' time
		// cur_pt.header.stamp = start + d + ros::Duration(cs.t);
		cur_pt.header.stamp = obs_start_time + ros::Duration(cs.t);
    	cur_pt.header.frame_id = "/map";
		// put current traj point into traj
		completeTraj_msg.traj[i] = cur_pt;
	}

	return completeTraj_msg;
}

void Planner_c::loadSingleMP(FILE* fp, char mp_name[]) {
	double x;
	double y;
	double theta;
	double t;
	fscanf(fp, "%lf %lf %lf %lf",&x, &y, &theta, &t);
	mapCell::carState* cur = new mapCell::carState();
	cur->x = x;
	cur->y = y;
	cur->theta = theta;
	cur->t = t;
	if (strcmp("straight",mp_name)==0) {
		straightMP.push_back(cur);
	} else if (strcmp("leftTurn",mp_name)==0) {
		leftMP.push_back(cur);
	} else if (strcmp("rightTurn",mp_name)==0) {
		rightMP.push_back(cur);
	} else {
		ROS_WARN("[planner] Motion Primitive loading error");
	}
	//cout<<x<<" "<<y<<" "<<theta<<" "<<t<<endl;
}

void Planner_c::loadMP() {
	// check if there is mp param
	// if only test this function, need to
	// rosparam set /MPRIM_FILE /home/ubuntu/catkin_ws/src/student1707024/project3/an_dynamic_planner/config/mprim.txt
	while (!ros::param::has("MPRIM_FILE")) {
    	ROS_WARN("[planner] sleeping while waiting");
    	ros::Duration(0.1).sleep();
	}

	string fname;
	ros::param::get("/MPRIM_FILE", fname);
	ROS_DEBUG("[planner] %s", fname.c_str() );

	FILE* fp = fopen(fname.c_str(), "r");
	//cout<<"fname"<<fname.c_str()<<endl;
	if (fp == NULL) {
		ROS_WARN("[planner] No motion primitive file!  Exiting!!");
		fclose(fp);
		return;
	}

	int arg_sz = 0;
	char mp_name[20];
	int row_numb;
	// there are 3 motion primitives
	int mp_number = 3;
	for (int i = 0; i < mp_number; i++) {
		arg_sz = fscanf(fp, "%s %i",&mp_name, &row_numb);
		for (int j = 0; j < row_numb; j++) {
			loadSingleMP(fp, mp_name);
		}
	}
	// put all MP in one single vector, order is straight, left, right
	allMP.push_back(&straightMP);
	allMP.push_back(&leftMP);
	allMP.push_back(&rightMP);
	cout<<"s: "<<straightMP.size()<<" l: "<<leftMP.size()<<" r:"<<rightMP.size()<<endl;
	//cout<<mp_name<<" "<<row_numb<<endl;
	//arg_sz = fscanf(fp, "%s %i",&mp_name, &row_numb);

}

// call back function of goal subscriber
void Planner_c::goalsub_callback(const geometry_msgs::PoseStamped& msg) {
	// the car state at goal position
	mapCell::carState goal_state; 
	// position of goal
	goal_state.x = msg.pose.position.x;
	goal_state.y = msg.pose.position.y;
	// convert quanternion to yaw
	geometry_msgs::Quaternion quan;
	quan.x = msg.pose.orientation.x;
	quan.y = msg.pose.orientation.y;
	quan.z = msg.pose.orientation.z;
	quan.w = msg.pose.orientation.w;
	goal_state.theta = tf::getYaw(quan);
	// cout<<"goal yaw: "<<yaw<<endl;
	// fill in the map cell
	goal_cell = new mapCell(goal_state,cell_size);
	// cout<<goal_state.x<<"goal_callback "<<goal_state.y<<endl;
	// set the flag to true
	goal_recieved_ = true;
	// cout<<"goal check"<<endl;
}

void Planner_c::posesub_callback(const geometry_msgs::PoseStamped& msg) {
	// the car state at start position
	mapCell::carState start_state;
	// position of start
	start_state.x = msg.pose.position.x;
	start_state.y = msg.pose.position.y;
	// convert quanternion to yaw
	geometry_msgs::Quaternion quan;
	quan.x = msg.pose.orientation.x;
	quan.y = msg.pose.orientation.y;
	quan.z = msg.pose.orientation.z;
	quan.w = msg.pose.orientation.w;
	start_state.theta = tf::getYaw(quan);
	// cout<<"start yaw: "<<yaw<<endl;

	start_cell = new mapCell(start_state,cell_size);
	// cout<<start_state.x<<"pose_callback "<<start_state.y<<endl;
	// set the flag to true
	pose_recieved_ = true;
	// cout<<"pose check"<<endl;
}

void Planner_c::mapsub_callback(const an_messages::lanes& msg) {
	int lane_numb = msg.lanes.size();
	// cout<<"there are "<<lane_numb<<" lanes"<<endl;

	double total_w = 0;
	double total_l = 0;
	double most_left_lane = numeric_limits<double>::max();
	double most_right_lane = numeric_limits<double>::min();
	// set the lanes
	// make assumptions here that the lane message is correct and exactly the same as expected
	// DID NOT do any safty check here, may add safty check later
	for (int i = 0; i < lane_numb; i++){
		an_messages::lane cur_l = msg.lanes[i];
		total_w = total_w + cur_l.width;
		total_l = cur_l.leftedge[1].x;

		if (cur_l.leftedge[0].y < most_left_lane) {
			most_left_lane = cur_l.leftedge[0].y;
		}
		if (cur_l.rightedge[0].y > most_right_lane) {
			most_right_lane = cur_l.rightedge[0].y;
		}
	}

	constructGlobalMap(total_w, total_l, most_left_lane, most_right_lane, cell_size);

	map_recieved_ = true;
	// cout<<"map check"<<endl;
}

void Planner_c::obstacle_callback(const an_messages::obstacles& msg) {
	// Process the obstacles
	int obs_numb = msg.obs.size();
	double testTime = ros::Time::now().toSec();
	cout<<"start obs: "<<testTime<<endl;
	for (int i = 0; i < obs_numb; i++) {
		an_messages::obstacle obs_msg = msg.obs[i];
		mapCell::obsState cur_obs;
		// get car length and width
		double length = obs_msg.length;
		double width = obs_msg.width;
		cur_obs.innerR = obs_msg.inner_radius;
		cur_obs.outerR = obs_msg.outer_radius;
		// develop the other two circles as obsCell
		double circle_offset = length/3;
		double circle_r = sqrt(pow(length/6, 2) + pow(width/2, 2));
		
		cur_obs.threeCircleR = circle_r;
		double start_time = obs_msg.path[0].traj[0].header.stamp.toSec();
		obs_start_time = obs_msg.path[0].traj[0].header.stamp;

		for (int j = 0; j < obs_msg.path[0].traj.size(); j++) {
			cur_obs.x = obs_msg.path[0].traj[j].position.x;
			cur_obs.y = obs_msg.path[0].traj[j].position.y;
			double cur_time = obs_msg.path[0].traj[j].header.stamp.toSec() - start_time;
			processObstacles(cur_obs, cur_time);
		}
		
		// double t0 = obs_msg.path[0].traj[0].header.stamp.toSec();
		// double t1 = obs_msg.path[0].traj[1].header.stamp.toSec();
		// double t10 = obs_msg.path[0].traj[100].header.stamp.toSec();
		// cout<<"t1 - t0: "<<t1 - t0<<" t10 - t0: "<<t10 - t0<<endl;
	}
	double testTime2 = ros::Time::now().toSec();
	cout<<"end obs: "<<testTime2<<endl;
	cout<<"duration: "<<testTime2 - testTime<<endl;
	obstacle_recieved_ = true;
	// cout<<"obs check"<<endl;
}

bool Planner_c::Init_(void) {
	ROS_DEBUG("[planner] Entering Init");
	// load the motion primitives
	Planner_c::loadMP();
	// subscribe to goal topic 
	ros::Subscriber goal_sub = nh_.subscribe("goal",1,&Planner_c::goalsub_callback,this);
	// subscribe to pose topic
	ros::Subscriber pose_sub = nh_.subscribe("pose",1,&Planner_c::posesub_callback, this);
	// subscribe to lanes topic(get map)
	ros::Subscriber mapLanes_sub = nh_.subscribe("lanes",1, &Planner_c::mapsub_callback, this);
	// subscribe to obstacle topic
	ros::Subscriber obst_sub = nh_.subscribe("obstacles",1, &Planner_c::obstacle_callback, this);
	// rosspin to get the recieved msgs
	ros::Rate loop_rate(150);
	// wait until got all the information
	while (ros::ok() && (!goal_recieved_ || !pose_recieved_ || !map_recieved_ || !obstacle_recieved_)) {
		ros::spinOnce();
    	loop_rate.sleep();
	}
	return true;
}

void Planner_c::Loop_(void) {
	int loop_rate = 150;
	ros::Rate loop(loop_rate);
	// ros::Rate ltest(1);
	an_messages::trajectory traj_msg;
	while (ros::ok()) {
		if (new_plan_ && pose_recieved_ && goal_recieved_ && obstacle_recieved_ && map_recieved_) {
			ROS_WARN("[planner] Started planning");
			new_plan_ = false;
			// add a duration to the cycle, so that when rviz will recieve the obstacles before moving the ego vehicle
			ros::Duration d(4.6);
			traj_msg = Planner_c::plan(d);
			ROS_WARN("[planner] Finished planning");
			ros::Publisher traj_pub = nh_.advertise<an_messages::trajectory>("planner_trajectory",10);
			// ltest.sleep();
			d.sleep();
			// cout<<"finish waiting"<<endl;
			traj_pub.publish(traj_msg);
			ROS_WARN("[planner] Published trajectory");
			cout<<"total length of path is: "<<total_length<<endl;
		}
		ros::spinOnce();
		loop.sleep();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "planner");
	ROS_DEBUG("[planner] start");
	Planner_c planner;
		if (planner.Init_()) {
		ROS_DEBUG("[planner] Entering loop");
		planner.Loop_();
	}
}