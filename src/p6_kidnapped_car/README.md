# **Kidnapped Vehicle - Particle Filter **  
The goals / steps of this project are the following:
* Using C++ build a particle filter.
* The state vector of the particle filter is position in the x and y directions and heading
* Initial GPS coordinates will be given to initialize the filter, but remaining localization will utilize measurements (presumably lidar) to surrounding landmarks that can then be matched to a map that known a priori.  
* Test the filter by connecting it to the term 2 simulator which will provide the measurements
* Filter must be performant enough to run in less than 90 seconds

[map]: ./data/map.png "map"
[success]: ./data/success.png "success"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1965/view) individually and describe how I addressed each point in my implementation.  

### 1. Does your particle filter localize the vehicle to within the desired accuracy?

My project successfully finished with an accuracy of x: .128 m y: .122 m and yaw of 0.007 radians (0.4 degrees).

![alt text][success]


### 2. Does your particle run within the specified time of 100 seconds?

As can be seen above, my code runs is about 49 seconds, significantly beating the 100 seconds requirement. 

### 3. Does your code use a particle filter to localize the robot?

This it does. I rewrote the entire particle filter class and associated type definitions. I did this because some of the steps in the template repository didn't make sense to me and so I set the new class in a way that did. As far as I can tell I implemented all aspects of the particle filter correctly as they were given lectures.

One item that didn't make sense is how noise is added into the process.  `main.cpp` specifies that measurements coming from the sim were noiseless. However instead of adding the noise immedately, it was added during the prediction step. This seemed odd to me from an architectural standpoint. I wasn't sure if adding the noise there was meant to simulate real noise a real measurement would have or if spread the samples apart to avoid them all collapsing in on one another. [This video](https://youtu.be/kNthLZTHDIM?t=25) says we should be adding noise to the speed and yaw rate measurements, but no values are given for the sigma.  All in all I was happy with this project and now have a much improved understand of particle filters.