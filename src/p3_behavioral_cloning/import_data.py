
import csv
import sys


def fix_path(current_filename, key, new_path):
  '''
  replace the prefix of a path with a new path.  The prefix is defined as that 
  part of the path before the key.  i.e. for /this/is/my/path.jpg, if the key is
  'my' then the prefix is '/this/is' and it would be replaced with the new path 
  '''
  ind = current_filename.find(key)
  new_filename = new_path + "/" + current_filename[ind:]
  return new_filename

def process_line(line, log_dir, new_path):
  '''
  process a single from driving_log.csv. each line from the csv is coverted to
  three "samples"-- one for each camera.  A sample is a tuple consisting of a 
  corrected path to an image and steering angle (also adusted as needed).  
  '''
  images = []
  measurements = []

  center_image_file = fix_path(line[0], log_dir, new_path)
  left_image_file = fix_path(line[1], log_dir, new_path)
  right_image_file = fix_path(line[2], log_dir, new_path)
  
  steering_angle = float(line[3])
  correction = .2

  images.append(center_image_file)
  measurements.append(steering_angle)
  
  images.append(left_image_file)
  measurements.append(steering_angle + correction)
  
  images.append(right_image_file)
  measurements.append(steering_angle - correction)

  return images, measurements

def import_data(log_dir_list, new_path):
  '''
  import data from a list of directories.  new_path is the new prefix for the 
  images that will be read in.
  '''    
  image_files = []
  measurements = []
  
  for log_dir in log_dir_list:
    with open(log_dir + "/driving_log.csv") as csvfile:
      reader = csv.reader(csvfile)
      for line in reader:
        new_image_files, new_measurements = process_line(line, log_dir, new_path)
        image_files = image_files + new_image_files
        measurements = measurements + new_measurements
              
  return image_files, measurements
    
