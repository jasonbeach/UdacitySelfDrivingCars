# **Extended Kalman Filter**  
The goals / steps of this project are the following:
* Using C++ build an EKF.
* The state vector of the EKF is position and velocity in the x and y directions
* Radar and Lidar will be used for measurement updates  
* Test the filter by connecting it to the term 2 simulator which will provide the measurements
* Filter must achieve a RMSE of less that [.11, .11, 0.52, 0.52]

[image1]: ./data/dataset1.png "Dataset1"
[image2]: ./data/dataset2.png "Dataset2"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

### 1. Code must compile without errors with cmake and make.

My project includes the following files:
* `FusionEKF.h` Header file containing the class declaration for the EKF
* `FusionEKF.cpp` Source file containing the definitions or all of the EKF class functions
* `tools.h` Header file containing utility function declarations for calculating RMSE
* `tools.cpp` Source file containing utility function definitions
* `measurement_package.h` Header for measurement struct defintion
* `main.cpp` Main file that that defines the websocket connection and EKF callback
* `json.hpp` The excellent nlohmann json library
* `system_include` Contains the Eigen Linear Algebra library
* `writeup_report.md` summarizing the results

This code compiles warning and error free. 

### 2. Code must against dataset 1 with an RMSE of less than [.11 m, .11 m, 0.52 m/s, 0.52 m/s]
Results of Dataset 1: [ 0.0967 m, 0.0852 m, 0.4005 m/s, 0.4501 m/s]
![alt text][image1]

Results of Dataset 2: [0.0930 m, 0.0915 m, 0.4457 m/s, 0.4625 m/s]
![alt text][image2]


### 3. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

I felt that separate `FusionEKF` and the `kalman_filter` classes was a little awkward and didn't make it obvious what the responsibilities for each class were. For simplicity I combined them. The FusionEKF manages the state of the EKF and has the functions for the prediction step and radar and lidar measurement updates.

### 4. Your Kalman Filter algorithm handles the first measurements appropriately.

The `FusionEKF` class has logic to intialize the state vector under two conditions: 1) On reception of the first measurement update. 2) When `dt` between measurements is excessively large.  This should only happen when datasets are swapped. 

### 5. Your Kalman Filter algorithm first predicts then updates.

This it does.

### 6. Your Kalman Filter can handle radar and lidar measurements.

This it does.  It does a standard Kalman filter update for lidar, and and EKF update (including calculation of the Jacobian).

### 7. Your algorithm should avoid unnecessary calculations.

This it does.  