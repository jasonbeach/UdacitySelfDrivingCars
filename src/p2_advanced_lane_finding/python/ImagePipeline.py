import cv2
import numpy as np
import os
import time

class FrameSource:
  """ Wrapper around OpenCV videoreader """
  def __init__(self, dev_string = None):
    self._reader = cv2.VideoCapture(dev_string)
    self.frame_size = {}
    self.frame_size = (0,0)
    self.frame_rate = 0
  
    if not self._reader.isOpened():
      raise ValueError('FrameSource: Error opening file: {}'.format(dev_string))
      return
    
    self.frame_size = (int(self._reader.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                       int(self._reader.get(cv2.CAP_PROP_FRAME_WIDTH)))
    self.frame_rate = self._reader.get(cv2.CAP_PROP_FPS)  

  def IsOpen(self):
    return self._reader.isOpened()

  def GetFrame(self):
    ret, frame = self._reader.read()
    if ret:
      return frame
    return None

  def GetFrameSize(self):
    return self.frame_size

  def GetFrameRate(self):
    return self.frame_size

  def Reset(self):
    self._reader.set(cv2.CAP_PROP_POS_FRAMES, 0)

  def Close(self):
    self._reader.release()

class FrameSink:
  """ Wrapper around OpenCV videowriter """
  def __init__(self, dev_string, frame_size, frame_rate):
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    self._writer = cv2.VideoWriter(dev_string, fourcc, frame_rate, frame_size)
    if not self._writer.isOpened():
        raise ValueError("FrameSink: Error opening {} for output".format(dev_string))

  def IsOpen(self):
    return self._writer.isOpened()

  def WriteFrame(self, frame):
    self._writer.write(frame)

  def Close(self):
    self._writer.release()


class ImagePipelineParams:
  """simple struct for holding pipeline params """
  def __init__(self):
    self.video_filename = "video.mp4"
    self.trackbar_window = "trackbars"
    self.image_window = "image"
    self.frame_rate_hz = 30
    self.loop_video = False
    self.record_video = False
    self.show_trackbars = False

class ImagePipeline:
  """ image pipeline object """
  def __init__(self, params):
    self.params = params
    self.video_reader = FrameSource(params.video_filename)
    self.video_writer = None
    if params.record_video:
      out_file = self._gen_out_file(params.video_filename)
      self.video_writer = FrameSink(out_file, self.video_reader.frame_size, self.video_reader.frame_rate)
      

  def GetFrameSize(self):
    return self.video_reader.frame_size
    
  def Run(self, frame_callback):
    if self.params.show_trackbars:
      cv2.createTrackbar("Frame Rate (Hz)", self.params.trackbar_window, self.params.frame_rate_hz, 50, self._update_framerate)

    while True:
      start_time = time.time()
      input_frame = self.video_reader.GetFrame()

      if input_frame is None:
        if self.params.loop_video:
          self.video_reader.Reset()
          continue
        else:
          break;
      
      output_frame = frame_callback(input_frame)
      cv2.imshow(self.params.image_window, output_frame)

      if self.video_writer is not None and self.video_writer.IsOpen():
        self.video_writer.WriteFrame(output_frame)
      
      stop_time = time.time()
      process_time = stop_time - start_time
      
      delay_time = 0 # infinite
      if self.params.frame_rate_hz > 0: 
        frame_period = 1.0 / self.params.frame_rate_hz
        delay_time = max(.001, (frame_period - process_time) ) * 1000.0

      key_val = cv2.waitKey(int(delay_time))
      if key_val == 27:
        break
    
    cv2.destroyAllWindows()

  def _gen_out_file(self, in_file):
    full_file = os.path.abspath(in_file)
    dir_name = os.path.dirname(full_file)
    file, ext = os.path.splitext(in_file)
    return file + "_out" + ext

  def _update_framerate(self, val):
    self.params.frame_rate_hz = val
  
  