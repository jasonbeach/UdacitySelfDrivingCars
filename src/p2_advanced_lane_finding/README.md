# **Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)
[image0]: ../../test_images_output/project2/chessboard_corners.jpg  "distorted"
[image1]: ../../test_images_output/project2/chessboard_corners_undistorted.jpg "Undistorted"
[image2]: ../../test_images/project2/test_images/signs_vehicles_xygrad.png  "Road Distorted"
[image2a]: ../../test_images_output/project2/signs_vehicles_xygrad_undistort.png  "Road Undistorted"
[image3]: ../../test_images_output/project2/signs_vehicles_xygrad_color_thresh.png "Threshold Example"
[image3a]: ../../test_images_output/project2/signs_vehicles_xygrad_binary_thresh.png "Binary Example"
[image4]: ../../test_images_output/project2/signs_vehicles_xygrad_warped.png  "Warp Example"
[image5]: ../../test_images_output/project2/signs_vehicles_xygrad_warped_annotated.png "Fit Visual"
[image6]: ../../test_images_output/project2/signs_vehicles_xygrad_fully_annotated.png  "Output"


## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.    

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for finding the chessboard corners in contained in this step is contained in the function `extractPointsFromImage` in [calibrate.cpp](../7_camera_calibration/calibrate.cpp).
  
I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z = 0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv::calibrateCamera()` function in [checkerboard.cpp](../7_camera_calibration/checkerboard.cpp) (line 43).  I applied this distortion correction to the test image using the `cv::undistort()` function (line 54). The image before distortion correction: 

![alt text][image0] 

and after:
![alt text][image1]

To facilitate using the K and D matrices in other software, the last step of the program was to dump them to a YAML configuration file that could be read in by other programs.

### Pipeline (single images)
I will describe the steps of the pipeline using this image:
![original image][image2]
#### 1. Correct image for distortion.
Using the calibration data I obtained in the previous section, I corrected the image
for distortion:

![alt text][image2a]

#### 2. Apply thresholds

I experimented with several threshold methods including the Sobel operator in the x and y directions, gradient magnitude and orientation, Laplace operator and transforming the image to the HSL colorspace.  I found that a combination of using the Sobel oeprator in the x directions and thresholding on the Saturation and Lightness channels. In testing with indiviual frames I applied thresholds to the saturation and lightness channels and then bitwise-or'd the resulting images together with the output of the sobel thresholded image.  This can be seen in lines 177 - 186 of [warped_main.cpp](../9_advanced_computer_vision/warped_main.cpp).  With the video this turned out to not be robust to shadows. I found that I could use the lightness channel to filter out the shadows by bitwise-anding it with the saturation channel output.  This can be in line 401 of [AdvancedLaneFinder.cpp](./AdvancedLaneFinder.cpp) 

Using green for pixels that were obtained from the Sobel operator and blue for pixels that were obtained from the color thresholding we get:
![alt text][image3]

The two channels for this can be bitwise-or'd to get a single binary channel.

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

Instead of hardcoding the source and destination points, I used OpenCV's GUI functions to display an image and then click on the image for source points. I used this image:
![straight_lines1.jpg](../../test_images/project2/test_images/straight_lines1.jpg)

Using the lane lines I picked four points in a trapezoid shape.  To get the destination points, I calculated the length of the two hypotanuses of the trapezoid and then used those lengths to project the bottom two points on the trapezoid up vertically to form a rectangle.  I am certain this is not the best way to do this, but it seemed to give ok results. The code for this turned out to be rather ugly but is found in the handle_mouse_click function in [perspective.cpp](../7_camera_calibration/perspective.cpp)

Unfortunately I did not record the source points I used.  The program from `perspective.cpp` simply dumped the perspective transform matrix to a yaml file that I could then read in in other programs.  The source and destination points I used resulted in:

![alt text][image4]

which is ok, but not perfect by any means.  At a future time I will go back an use the hard-coded numbers given in the template write up of this project.

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The code for finding the lane pixels starts at line 61 of [warped_main.cpp](../9_advanced_computer_vision/warped_main.cpp) and goes through line 172. In that pipeline on the histogram search method is implemented. In the video pipeline, I augment this with using the fitted curve to predict where the lane lines should be. This is shown in:

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The radius of curvature for each lane line is caluated in line 142 of at line 61 of [warped_main.cpp](../9_advanced_computer_vision/warped_main.cpp).  Once the radius is calculated for both lines, I average them together to come up with an overall lane radius of curvature. 


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Plotting the resulting lane starting in at lines 230 - 247 of [warped_main.cpp](../9_advanced_computer_vision/warped_main.cpp)
  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

As you can probably see in [warped_main.cpp](../9_advanced_computer_vision/warped_main.cpp), the code quality was shall we say not greatest--it worked, but was all kind of jumbled together and really wasn't architected in any sense.  For the video, I completed refactored the entire pipeline.  A couple of nice things fell out of this.  I now have the [`ImagePipeline`](../common/ImagePipeline.hpp) class along with `FrameSource`s and `FrameSink`s that can trivally be reused on future projects process frames in a video.  FrameSources is implement generically so creating an implentation that grabs video from a camera instead of a video file is trivial and then swapping between live video and recorded video can be done as a configuration parameter. 
All of the actual lane detection occurs starting in line 310 of [AdvandedLaneFinder.cpp](./AdvancedLaneFinder.cpp)

Here's a [link to my video result](https://youtu.be/6rY7u-j04Uk)

---

### Discussion

This was a great project as you can tell all of the code is written in C++ which was done for three reasons, 1) performance, 2) that's what would be used in a real self-driving car and 3) to force myself to get more more familiar with OpenCV and get industrial grade experience with it.

OpenCV is fantastic.  Using it's GUI functionality, I was able to create trackbars that allowed me to quickly tune and experiment with thresholds in real-time as a video was playing.

Getting the non-zero pixels in the warped image seems weird to me since the upper portion of the image is so stretched out.  I really wanted to try an algorithm called Recursive-RANSAC to detect the lines before warping them and then just warping a small set of points from a generated curve. Unfortunately I don't have the time to do it now.

My pipeline works quite well on the project video, but does pretty abysmal on the challenge and the harder challenge video.  I didn't have time to look in-depth at the cause, but it a cursory look seemed to indicate that the perspective transform was off. It would be interesting to use the hardcoded transform and see if that improves things. 

### Building

if you want to build the software, you will need at least version 3.14 of CMake, and a newer version of GCC that has support for C++17's std::filesystem. I used GCC 9.2. I also assume that OpenCV (I used 3.4.6) is already installed.  All remaining dependencies are downloaded by CMake and unpacked in the build directory.

1. clone repository
2. inside the cloned repository create `build` and `install` directories
3. cd into the build directory
4. `cmake ..`
5. `make -j8`
6. `make install` # no sudo needed as it installs in the install directory.
7. the test images and videos are copied into the install directory
8. passing the `-h` flag to most of the programs will display config options.  
9. The executable for project 2 is `p2_advanced_lane_finding`
10. By default it doesn't need any flags, but `-f {video file}` can be used to process a different video file and `-t` can be used to make the threshold adjustment trackbars appear. `-w` causes the warped images to be shown in the output frame. 

