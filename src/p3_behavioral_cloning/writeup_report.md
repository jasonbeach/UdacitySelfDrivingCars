# **Behavioral Cloning**  
The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: https://devblogs.nvidia.com/parallelforall/wp-content/uploads/2016/08/cnn-architecture-624x890.png "Model Visualization"
[image2]: ./images/center_2020_02_26_06_34_06_624.jpg "Center driving"
[image3]: ./images/center_2020_02_26_06_34_09_755.jpg "Recovery Image"
[image4]: ./images/center_2020_02_26_06_34_10_568.jpg "Recovery Image"
[image5]: ./images/center_2020_02_26_06_34_11_841.jpg "Recovery Image"
[image6]: ./images/mse_loss.png "mse loss"
[video1]: https://youtu.be/yQx4RlXdRwI "Working network without dropout"
[video2]: https://youtu.be/EE0jxyq_n2Y "Final video with dropout"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* `model.py` containing the script to create and train the model
* `drive.py` for driving the car in autonomous mode
* `model.h5` containing a trained convolution neural network 
* `writeup_report.md` summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my `drive.py` file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```
in one terminal and running the simulator in another.

#### 3. Submission code is usable and readable

The `model.py` file contains the code for training and saving the convolution neural network. To use it you would need to modify to the paths in function `main`. There are some assumptions on the structure of those paths making the correcting of them somewhat brittle. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of the same convolution neural network that Nvidia used in their [paper](http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) with 3x3  and 5x5 filter sizes and depths between 32 and 128 (model.py lines 48-52) 

The model includes RELU layers to introduce nonlinearity (code line 47-51), and the data is normalized in the model using a Keras lambda layer (code line 45). 

#### 2. Attempts to reduce overfitting in the model

In the final run, I did add a dropout layer to protect against overfitting, but I was somewhat surprised it didn't make things that much better from my previous run without the dropout layer.  With the data I had, the car was able to successfully drive the course when trained with and without the drop out layer.

The only method I ended up using for protecting against overfitting was to limit the number of epochs during training. I had planned on adding a dropout layer, but the model was able to sufficently drive the car without so it was not added. 

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 19-35). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 58).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road. I also tried to generalize the data by collecting several laps in both the clockwise and counter-clockwise directions.  I did not flip the images to augment the data.  I thought it would be more robust to just collect more data as that was quite easy.  I'm sure there are cases where collecting more data is difficult and augmenting the data by flipping, etc. would be more applicable.
The list of data collects is in `model.py` (lines 20-23) with each directory listed representing a single recorded run.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to start with a network that was known to work and accordingly I went straight to the model that Nvidia used. Had I enough time I probably would've tried the LeNet network as well.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I was surprised that I didn't really appear to have an overfitting issue.  Limiting the number of epochs was sufficient on training and for the most part both the mean squared error of the training set and validation set monotonically decreased.

The final step was to run the simulator to see how well the car was driving around track one. In the final run that worked, the car definitely favored the left side of the road but managed to stay on the road. 

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (`model.py` lines 45-58) consisted of a convolution neural network with the layers and layer sizes show here: 

![alt text][image1]

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image2]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to .... These images show what a recovery looks like starting from ... :

![alt text][image3]
![alt text][image4]
![alt text][image5]

After the collection process, I had 62,646 number of data points. I then preprocessed this data by normalizing it and mean shifting it via:
  

&ensp;&ensp;&ensp;&ensp;`x = (x / 255.0) - 0.5`

I noticed in some of the examples, this was done instead via:

&ensp;&ensp;&ensp;&ensp;`x = (x / 127.5) - 1.0`  

I'm not sure what it's trying to accomplish so I didn't use it. 

I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 5 as evidenced by 

![alt text][image6]

where we can see that the training and validation mse have started to level out or slightly increase. I used an adam optimizer so that manually training the learning rate wasn't necessary.

There are two final videos - one where the network uses dropout during training and one where it doesn't

[video1]

[video2]
