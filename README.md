# Extended Kalman Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

In this project the goal is to implementing an Extended Kalman Filter with C++. The kalman filter estimate the state of a moving object of interest with noisy LIDAR and RADAR measurements, while  obtaining RMSE values that are very low. The communication between the project and the simulator is done using WebSocket.

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

The WebSocket and other initial info regarding dependencies  installation can be found  [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

The Sensor Fusion General Flow:
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/general_flow.jpg?raw=true)

Prerequisites and Executing
---

This project requires the following dependencies:

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Build Instructions:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

Running the project:
1. Run the project main file: `./ExtendedKF `
2. You will get the following output:
 `Listening to port 4567. Connected!!! `
 3. Run the Simulator on `project 1` and start the process on one of the datasets.

Process results
---

The simulator final state after running the EKL:

Dataset 1              |  Dataset 2
:---------------------:|:---------------------:
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/dataset_1.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/dataset_2.jpg?raw=true)

The Extended Kalman Filter Residual error is calculated by mean squared error, and the accuracy is very good (low) for both datasets:

| Input |   MSE for dataset 1   | MSE for dataset 2   |
| ----- | ------- | ------- |
|  px   | 0.0973 | 0.0726 |
|  py   | 0.0855 | 0.0965 |
|  vx   | 0.4513 | 0.4216 |
|  vy   | 0.4399 | 0.4932 |

The Kalman Filter algorithm handles the first measurements appropriately. The measurement is handled at src/FusionEKF.cpp from line 59 to line 98

The estimation part is being done by two steps: the algorithm first predicts then updates. Predict process can be found at src/FusionEKF.cpp from line 114 to line 140. Update process can be found right after it at lines 147-160.
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/loop.jpg?raw=true)

The project can handle both RADAR and LIDAR measurements. For experience, I ran the process each time for only one of the inputs (lidar / radar), observing how the estimations change when running against a single sensor type. Here are the result:

only LIDAR               |  only RADAR
:---------------------:|:---------------------:
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/radar_only.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/laser_only.jpg?raw=true)
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/radar_only_2.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/laser_only_2.jpg?raw=true)

Its easy to see that both results have higher RMSE than when running the process with both of the inputs together. The LIDAR  screw up a large part of the way, giving the worst result.

The algorithm avoid unnecessary calculations. For example: The calculation of the Q matrix at EKF.cpp, skipping the 'prediction' step if the measurments came in the same time, and more.
The code is clean and readable, including verification for dividing by Zero and wrong inputs.

The video below shows what the simulator looks like when running with this Kalman filter project to track the object.
* LIDAR measurements are red circles
* RADAR measurements are blue circles with an arrow pointing in the direction of the observed angle
* Estimation markers are green triangles

[![video output](https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/dataset_1.gif)]
