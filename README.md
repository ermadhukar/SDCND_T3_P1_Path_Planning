# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.
In a nutshell, it should emulate a real driver behavior as close as possible.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


---

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
## Implementation

Based on the skeleton code provided in the project, path planning code has been implemented in the main.cpp. Apart from given header file spline.h has been included in the code to get a smooth lane shift trajectory. Path planning code has been segregated in following parts: 

> Setting vehicle initial state and parametrization

> Predicting the traffic

> Creating the trajectory

While doing so, vehcile need to operate within speed limit without crossing suggested acceleration and jerk values. 


## Initial Condition: 
Vehicle is started in the mid lane with zero initial velocity.


	// Start in lane 1
	int lane = 1;

	// Reference velocity to target
	double ref_vel = 0.0; // mph


Vehicle Max Speed is kept at slightly less than the speed limit of 50 and acceleration has been kept at 0.21

			double acc = 0.21; // acceleration is tunned to reduce sudden jerk
			double max_speed = 48; // Max speed is less than 50, so that it doesn't cross speed limit
			

## Prediction: 
Here code analyze the vehicles in traffic location using Localization and Sensor Fusion data. Any vehicle which is in the same lane and less than 30m ahead will result in the vehcile deacceleration.

			bool too_close = false;
			bool car_left = false;
			bool car_right = false;

			for (int i = 0; i < sensor_fusion.size(); i++) {
				
				float d = sensor_fusion[i][6];

				// Car lane Identifying
				int car_lane;
				if (d >= 0 && d < 4) 
					{
					car_lane = 0;
					} 
				else if (d >= 4 && d < 8) 
					{
					car_lane = 1;
					} 
				else if (d >= 8 && d <= 12) 
					{
					car_lane = 2;
					} 
				else 
					{
					continue;
					}

				// Check width of lane, in case cars are merging into our lane
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx + vy*vy);
				double check_car_s = sensor_fusion[i][5];

				// If using previous points can project an s value out
				
				// check s values greater than ours and s gap
				check_car_s += ((double)prev_size*0.02*check_speed);

				int gap = 30; // car gap, m

				// Identify whether the car is ahead, to the left, or to the right
				if (car_lane == lane) 
					{
					// Another car ahead
					too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
					} 
				else if (car_lane - lane == 1) 
					{
					// Another car to the right
					car_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
					} 
				else if (lane - car_lane == 1) 
					{
					// Another car to the left
					car_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
					}
				}


Based on the driving condition and prediction, decision of vehicle speed control and lane change is taken. While taking decision vehicle state and traffic condition is been considered for safety and comfort.

				if (too_close) {
				// Lane shift decision

				if (!car_right && lane < 2) {
					// Lane change to right, if there is no car in right lane and car is not in the right most lane
					lane++;
				} else if (!car_left && lane > 0) {
					// Lane change to left, if there is no car in left lane and car is not in the left most lane
					lane--;
				} else {
					// Lane shift is not possible, slow down till lane shift happens
					ref_vel -= acc;
				}
			} else {
				if (lane != 1) {
					// Bringing back to mid lane
					if ((lane == 2 && !car_left) || (lane == 0 && !car_right)) {
						lane = 1;
					}
				}
				
				if (ref_vel < max_speed) {
					// reaching upto set speed limit, in space is available
					ref_vel += acc;
				}
			}

## Trajectory:
Once decision of the lane shift has been taken, trajectory of the vehicle lane shift is defined using spline.h.
It creates a smooth path for the vehicle to follow. 

			// Create a spline
			tk::spline s;

			// Set (x,y) points to the spline
			s.set_points(ptsx, ptsy);


## Compiling: 

> Clone the repo

> Make a build directory: mkdir build && cd build

> Compile the code using: cmake .. && make

> Run: ./path_planning

> Open the simulator and see the visualization


![](https://github.com/ermadhukar/SDCND_T3_P1_Path_Planning/blob/master/Image_T3P1.png)
