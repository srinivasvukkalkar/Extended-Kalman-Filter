# CarND-Extended Kalman Filter 
Self-Driving Car Engineer Nanodegree Program

[image1]: ./images/RMSE_results.jpg "RMSE"
[image2]: ./images/Simulator_Connection.jpg "Simulator Connection"
[image3]: ./images/Simulator_Screen.jpg "Simulator Screen"


## Overview

This project will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.Simulator and the EKF is communicated using WebSocket (uWebSockets).

## Dependencies

1. cmake >= 3.5
2. make >= 4.1
3. gcc/g++ >= 5.4
4. Udacity's simulator.

This project is completely done in project workspace provided by Udacity

## Compile and execute the project

1. Create the build directory: mkdir build
2. Navigate to build: cd build
3. CMake: cmake ..
4. Create executable (ExtendedEKF) : make
5. To Run the filter: ./ExtendedEKF

## Runing the Filter

After executing ExtendedEKF, output should look like

Listening to port 4567
Connected!!!

## Running the simulator with dataset 1

Simulator Connection

![alt text][image2]

Simulator screen :

![alt text][image3]

After running dataset 1 the RMSE values look like this

![alt text][image1]


## Acuracy

The output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] using Udacity Data
The EKF accuracy was:

Dataset 1 : RMSE <= [0.0974, 0.0855, 0.4517, 0.4404]



## Code Style

C++  [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

