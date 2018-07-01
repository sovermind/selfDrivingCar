# Project 4 -- Ego vechicle with dynamic obstacles
## By Jiarui Yang

### Summary of the project


#### Detail of the project
##### 1. Motion Primitive:
In order for the vechicle to move, it will generate three different motion primitives first. Going straight, change to left lane and change to right lane. [generateMP.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project3/an_static_planner/src/generateMP.cpp) uses Dubins Curve to generate the change lane motion. Thus it will ganruntee a smooth curve, reasonable curvature and lateral accelaration. Here assuming that the vechicle velocity is 25 m/s constant and the changing lane time is 4s. The final product of this file is a text file with the following format:
```
Motion primitive Name
x y theta time
```
After the motion primitives are generated, the text file will be put inside the config file in order for ROS to take this as the paramitive.
##### 2. subscribe functions
[planner.cpp](https://gitlab.com/sovermind/student1707024/blob/master/project3/an_static_planner/src/planner.cpp) will take care of all the subscribe functions.
`goal`: will get the goal information from `AutoNav` package. The goal represents the final car state.
`pose`: will get the start information from `AutoNav` package. The start represents the start car state.
the start and goal position will be saved and used as planning the trajectory.
`obstacle`: will get the 12 obstacle information from `AutoNav` package. 
`lanes`: will get the lane information from `AutoNav` package.
##### 3. setting up the whole map
The map will be divided into small cells defined by the `mapCell` class. The `mapCell` contains the following private fields: 
`cell_size`: define how large the cell is, it will be square.
`g`, `f`, `v`, `h`: define the cost information of the map, will be used when implementing ARA* algorithm.
`isObstacle`: bool variable will be set to false first and then set to true when the obstacle struct is initialized.
following public fields:
`carState`: the struct define the current car state within this mapCell. It is the true position of the car. Because eventually, when publishing the trajectory, I'll want to publish the true position of the car, not the cell location.
`obsState`: the struct define the obstacle information of this cell. It will turn the isObstacle flag to true. And will contian radius of the obstacle.
`two different constuctors`: one will construct with carState, another will construct with obsState.
`setter and getter:` for all the private values.
`row` and `col`: will define the position of the cell inside the map.
`parent`: used to record where does this cell come, and will be used to trace back to previous step.
The map will be a 2 D vector of `mapCell`. After get the information from the `lanes` topic, and using `cell_size`, it can divide the whole map into small cells.
##### 4. planning using ARA*
there are two functions `finalPathGenerate` and `ARAstar`. The `finalPathGenerate` will set up epsilon and call `ARAstar` repeatedly until the epsilon falls to 1. The epsilon will be multiplied by a decreasing factor during each loop. `ARAstar` will generate a path using current epsilon value. This value will be multiplied to the herustic value so that it will find the path quickly. When generating the next successor point, I used motion primitives saved before and also using obstacles to determine if it is a valid successor point. To determine the obstacle collison, I used three circles to represent the whole car body. And I assume the obstacle cars have the same dimension as the ego vechicle. Thus, by checking if the six circles' radius, I can determine if there's collision. If there is collision, I won't put that successor into the successor list.
##### 5. adding the time state to the planner
Different from project 3, now it will take time as additional state. Therefore, for obstacles, now for each time step, it will have different positions. To store and use this information, I used a map with key = time, value = vector of obstacles. When performing the collision detect, use the time at that time step, to search for the positions of all obstacles.
When tracing back the point of all motion primitives' points, use the current time step plus the corresponding increment of that motion primitive point. 
One last need to be sure is since the obstacles are published with a 5 seconds delay and will not be starting at the same time as the planner, need to take that time and sync with the planner's starting time. So that in Rviz, the planner's route will show up correctly.