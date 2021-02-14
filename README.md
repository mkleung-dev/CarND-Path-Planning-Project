# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[FSM]: ./image/FSM.png "Finite State Machine"
[optimal_lane]: ./image/optimal_lane.png "Optimal Lane"
   
### Simulator.
The Term3 Simulator which contains the Path Planning Project can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The car's localization and sensor fusion data are provided.
There is also a sparse map list of waypoints around the highway.
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
The car should be able to make one complete loop around the 6946m highway.
Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

---

## File Description

|Files|Description|
|:----|:----------|
|`src/main.cpp`|1. Communicates with the Term 3 Simulator receiving data measurements. <br /> 2. Call functions to plan the path.|
|`src/helpers.h`|Some Helper functions.|
|`src/waypointmap.h` <br /> `src/waypointmap.cpp`|Class to store waypoint data|
|`src/vehicle.h` <br /> `src/vehicle.cpp`|Class to store vehicle data.|
|`src/myvehicle.h` <br /> `src/myvehicle.cpp`|Class to store my vehicle data.|

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Basic Run Instructions

|Command|Description|
|:------|:----------|
|`./path_planning`|Using extended Kalman filter with both radar and Lidar measurement.|

---
## Data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Implementation

The implemenation is described in the followings.

### Finite State Machine

It is implemented in `MyVehicle::update_state()` in `myvehicle.cpp`.

The finite state machine is illustrated in the following figure.
![FSM]

The optimal lane is defined as the lane with the most far away vehicle ahead.
The following figure shows an example.
![optimal_lane]

### Update Action

It is implemented in `MyVehicle::update_action()` in `myvehicle.cpp`.

|State|Action|
|:----|:-----|
|Keep Lane|Compute target velocity according to the vehicle ahead.|
|Change to Left Lane|Update the target lane to the left lane.|
|Change to Right Lane|Update the target lane to the right lane.|
|Prepare to chagne to Left Lane|Compute target velocity according to the vehicle ahead.|
|Prepare to chagne to Right Lane|Compute target velocity according to the vehicle ahead.|

### Compute Trajectory

It is implemented in `MyVehicle::compute_trajectory()` in `myvehicle.cpp`.

1. Pass the following data in the `class spine` of [simple cubic spline interpolation library](http://kluge.in-chemnitz.de/opensource/spline/).
    - The previous path X, Y sent back from the simulators, or the current state XY.
    - The XY coordinate in the target lane 30 meters after the previous path or the current state.
    - All the XY coordinates are transformed to reference from the current vehicle position.

2. Use the `spine` object to generate XY coordinates whose seperations are specially computed so that they are 0.02 second apart.

3. Transform the XY coordinates to the global map coordinates.