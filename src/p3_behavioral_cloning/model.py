from import_data import import_data
import numpy as np
import cv2
import random
import sklearn
import sys
from sklearn.model_selection import train_test_split
import math
import matplotlib.pyplot as plt
import pickle

def main():
  # this is the prefix to the path within the docker container I used to do
  # the training in 
  new_path = "/tf/udacity/python/P4_behavioral_cloning"

  # I originally had data pre-divided into training, validation and test sets
  #  but since test sets weren't really used I just merged them into the 
  # training data
  data_dirs = ['my_data/cw', 'my_data/ccw', 'my_data/val', 'my_data/test',
              'my_data/cw2', 'my_data/ccw2', 
              'my_data/cw3', 'my_data/ccw3',
              'my_data/cw4', 'my_data/ccw4']

  image_files, angles = import_data(data_dirs, new_path)

  print("number of samples {}".format(len(image_files)))

  samples = list(zip(image_files, angles)) # this could probably be done better

  train_samples, validation_samples = train_test_split(samples, test_size=0.2)

  batch_size = 128

  training_generator = DataGenerator(train_samples, batch_size)
  validation_generator = DataGenerator(validation_samples, batch_size)

  from keras.models import Sequential
  from keras.layers import Flatten, Dense, Lambda, Cropping2D, Convolution2D, Dropout


  # This is the model from the Nvidia paper -  it's really simple.  I planned
  # on trying a dropout layer, but got it working well enough without it.  If 
  # there were additional time I would add a dropout layer.
  model = Sequential()
  model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
  model.add(Cropping2D(cropping=((70,25),(0,0))))
  model.add(Convolution2D(24,5,5,subsample=(2,2),activation='relu'))
  model.add(Convolution2D(36,5,5,subsample=(2,2),activation='relu'))
  model.add(Convolution2D(48,5,5,subsample=(2,2),activation='relu'))
  model.add(Convolution2D(64,3,3,activation='relu'))
  model.add(Convolution2D(64,3,3,activation='relu'))
  model.add(Flatten())
  model.add(Dropout(.2))
  model.add(Dense(100))
  model.add(Dense(50))
  model.add(Dense(10))
  model.add(Dense(1))

  model.compile(loss='mse', optimizer='adam')
  history_object = model.fit_generator(training_generator,
              steps_per_epoch=math.ceil(len(train_samples)/batch_size),
              validation_data=validation_generator, 
              validation_steps=math.ceil(len(validation_samples)/batch_size), 
              epochs=5, verbose=1)
    
  model.save('model.h5')
  
  with open('trainHistoryDict', 'wb') as file_pi:
        pickle.dump(history_object.history, file_pi)

  ### print the keys contained in the history object
  print(history_object.history.keys())
  #history = pickle.load(open('/trainHistoryDict', "rb")) 

  ### plot the training and validation loss for each epoch
  plt.plot(history_object.history['loss'])
  plt.plot(history_object.history['val_loss'])
  plt.title('model mean squared error loss')
  plt.ylabel('mean squared error loss')
  plt.xlabel('epoch')
  plt.legend(['training set', 'validation set'], loc='upper right')
  plt.show()





def load_image(image_file):
  '''
  load an image, bail out of the script if that fails, and convert the BGR
  images to RGB to match what drive.py does
  '''
  image = cv2.imread(image_file)
  if image is None:
    print("Failed to read: {}".format(image_file))
    sys.exit()

  return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def DataGenerator(samples, batch_size):
  '''
  
  '''
  num_samples = len(samples)
  while True:
    random.shuffle(samples)
    for offset in range(0, num_samples, batch_size):
      batch_samples = samples[offset:offset + batch_size]

      images = []
      angles = []
      for batch_sample in batch_samples:

        image = load_image(batch_sample[0])
        images.append(image)
        angles.append(batch_sample[1])

      X = np.array(images)
      y = np.array(angles)

      yield sklearn.utils.shuffle(X, y)


if __name__ == "__main__":
  main()

