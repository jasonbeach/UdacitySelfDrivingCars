# **Finding Lane Lines on the Road** 

This readme constitutes the writeup for project 1 for the Self-Driving Car Nanodegree.  Instead of using python, this project is written completely in C++. Instructions for building and using the software are at the bottom of this readme.  Annotated videos are on youtube:

[solid yellow left](https://youtu.be/5WLFWHbFqOU)

[solid white right](https://youtu.be/RU8oR3AAHhQ)

[challenge](https://youtu.be/cG-0g2MvilQ)

---

### Pipeline Description

The pipeline consists of 10 steps:
1. Convert image to gray scale
2. Apply Gaussian blur
3. Canny Edge Detection
4. Masking the image
5. Copying the image back into a three channel (BGR) matrix
6. Hough Transform
7. Apply gating criteria to lines from Hough Transform
8. Determine final lane position via RANSAC
9. Filter lane lines between images
10. Annotating the image

Steps 1 - 6 follow the steps from this chapter.  However instead of just averaging the lines from the output of the Hough transform, I wanted to reject as many of the false lines as possible. 

Assuming the car is correctly oriented within the lane, the angles of the correct lane lines are within a relatively narrow range. As such, the first gating criteria applied is the angle each line makes with the y-axis.  

After the grossly incorrect lines are rejected, the lines in a given image are refined and combined using a [random sample consensus (RANSAC)](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm which is well suited to this task.


Once the final lane lines are determined, the lines and all of the points from the hough transform (minus the grossly incorrect ones) are plotted over the frame being processed.


### 2. Limitations of the current pipeline

The association of lane-line detections between adjacent frames is pretty weak. The ransac also uses a line for a model which doesn't take lane curvature into account. Also at the time of submission, I'm still tracking down a bug in the RANSAC that intermittently causes the left lane line to not appear. I believe it is in the step where all the "inlier" points are fitted to the best model via least squares.  If you run the software and the left lane line does not appear, exiting the software and restarting fixes it.  

### 3. Potential improvements to the pipeline

If time would have permitted, I would've like to implement the [Recursive-RANSAC algorithm](https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=5194&context=etd) which presumably would provide better association between frames at a potentially lower computational cost than pure RANSAC.  


---
**Dependencies**

The software was written, compiled, and tested on Ubuntu 18.04. The primary dependency is [OpenCV](https://opencv.org/) (versions 3.2.0 and 3.4.4 have been tested) which made most tasks very straight forward. Additionally, the excellent [{fmt} library](https://github.com/fmtlib/fmt) is used for printing text to the screen and [CLI11](https://github.com/CLIUtils/CLI11) is used to parse command line arguments.  

CMake (at least v3.14) is used to build the software. Features of the more recent versions of CMake are used to automatically download these dependencies.  The repository also contains a Docker file that can be used to create a docker image to build and run the software.   

**Building** 

There are two methods to build the software:

_CMake_

If the correct versions of CMake and OpenCV are installed, then building is as simple as 

```
$> git clone https://github.com/jasonbeach/UdacitySelfDrivingCarsProject1.git
$> cd UdacitySelfDrivingCars
$> mkdir build
$> cd build
$> cmake -DCMAKE_BUILD_TYPE=Release ..
$> make -j8
$> make install
```
_Docker_

If you have docker installed, building is simply:
```
$> git clone https://github.com/jasonbeach/UdacitySelfDrivingCarsProject1.git
$> cd UdacitySelfDrivingCars
$> ./build.sh
```
Initially, this will build the needed docker image which will take several minutes. After the initial docker inage is built, the software is also built and installed. 

Both of these methods will create an `install` folder at the same level as the build folder. All of the compiled binaries as well as the test videos will be copied to the `install/bin` folder. 

_Usage_

If the software was not built with _Docker_, then to run the software:
```
$> cd ${clone dir}/install/bin
$> ./p1_finding_lane_lines [-ltcf] [path to video file]
```

With _Docker_, a convience script is provided to run the software:
```
$> cd ${clone dir}
$> ./p1_finding_lane_lines.sh [-ltcf] [path to video file]
```
The flags in both cases are the same and are optional. The `-l` flag causes the software to loop the video continuously and not record an output video.  The `-f` flag allows you to specify the video file to run.  If no video file is specified the software will attempt to use the `challenge.mp4` video. Note that in the Docker case the video file specified should be a relative path that is relative to the install/bin directory. The `-t` flags displays trackbars that can be used to adjust some parameters of the software. The `-c` flag changes the output image to show the output of the canny edge / hough transform output not annotated on the final image.
