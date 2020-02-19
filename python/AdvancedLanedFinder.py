from ImagePipeline import *
import cv2
import numpy as np
import sys

def ThresholdImage(image, min_val, max_val):
  ret, output = cv2.threshold(image, max_val, 0, cv2.THRESH_TOZERO_INV)
  ret, output = cv2.threshold(output, min_val, 255, cv2.THRESH_BINARY)
  return output

def alpha_filter(current_value, new_value, alpha):
  return alpha * new_value + (1.0-alpha) * current_value

class LaneLineParams:
  def __init__(self):
    self.xm_per_pix = 0
    self.ym_per_pix = 0
    self.margin = 0
    self.minpix = 0
    self.nwindows = 0

class AdvancedLaneFinderParams:
  def __init__(self):
    self.trackbar_window = ""
    self.K = None
    self.D = None
    self.M = None
    self.M_inv = None
    self.lp = LaneLineParams()
    self.min_sobel = 0
    self.max_sobel = 0
    self.min_s = 0
    self.max_s = 0
    self.min_l = 0
    self.max_l = 0
    self.show_trackbars = False
    self.show_warped = False
    self.initialized = False 

class LaneLineModel:
  def __init__(self):
    self.a = 0
    self.b = 0
    self.c = 0
    self.valid = False
  
  def CalcX(self, y):
    return self.a*y*y + self.b*y + self.c
  
  def CurvePoints(self, y_min, y_max, x_min, x_max, step = 20):
    curve = np.zeros( ((y_max - y_min) // step , 2) )
    ind = 0
    for y in range(y_min, y_max, step):
      x = self.CalcX(y)
      #if x >= x_min and x <= x_max:
      curve[ind, :] = [x, y]
      ind+=1
    return curve

  def __str__(self):
    return "a: {:.4f}, b: {:.4f}, c: {:.4f}".format(self.a[0], self.b[0], self.c[0])

class LaneLine:
  def __init__(self, lp = LaneLineParams(), starting_roi = None, side = "" ):
    self.lp = lp
    self._roi = starting_roi
    self._current_fit = LaneLineModel()
    self._best_fit = LaneLineModel()
    self.radius_of_curvature_meters = 0.0
    self._missed_detects = 0
    self._num_good_fits = 0
    self._detected = False
    self._do_reset = True
    self._best_x = np.array([])
    self.detected_pixels = np.array([])
    self.side = side


  def Update(self, warped_img):

    self.detected_pixels = self.find_points(warped_img)

    ret, self._current_fit = self.FitPoints(self.detected_pixels)

    if ret:
      current_curvature = self.compute_curvature_radius(self._current_fit, 720)

      alpha = .7
      self.radius_of_curvature_meters = alpha_filter(self.radius_of_curvature_meters, current_curvature, alpha)
    
      self._best_fit.a = alpha_filter(self._best_fit.a, self._current_fit.a, alpha)
      self._best_fit.b = alpha_filter(self._best_fit.b, self._current_fit.b, alpha)
      self._best_fit.c = alpha_filter(self._best_fit.c, self._current_fit.c, alpha)

      self._best_x = self._best_fit.CurvePoints(0, warped_img.shape[0], 0, warped_img.shape[1])
      self._num_good_fits += 1
      if self._num_good_fits > 5:
        self._missed_detects = 0
        self._do_reset = False
    else:
      self._missed_detects +=1
      if self._missed_detects > 10:
        print("failed to find lane line: {}".format(self._missed_detects))
        self._num_good_fits = 0
        self._do_reset = True

  def at(self, y):
    return self._best_fit.CalcX(y)

  def get_max_col(self, image):

    mask = np.zeros_like(image, np.uint8)
    new_image = image[self._roi]

    mask[self._roi] = 255

    image = cv2.bitwise_and(image, mask)

    summed = cv2.reduce(image, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)
    
    min_val, max_val, min_col, max_col = cv2.minMaxLoc(summed)
    
    return max_col[0]

  def find_points(self, image):


    start_col = 0
    margin = 0
    if self._do_reset:
      start_col = self.get_max_col(image)
      margin = self.lp.margin * 2.0
    else:
      start_col = self._best_fit.CalcX(image.shape[0])
      margin = self.lp.margin
    
    #print("start col: ", start_col)
    #image = cv2.line(cv2.linecv2.line.lineimage,(start_col,0),(start_col, image.shape[0]),255)
  
    window_height = image.shape[0] / self.lp.nwindows

    current_x = start_col - margin / 2
    current_y = image.shape[0] - window_height
    self._search_windows = np.empty((0, 5, 2), dtype=np.int32)

    #print("initial search window shape: {}".format(self._search_windows.shape))
    nonzero_points = np.empty((0,2), dtype=np.int32)
    finished = False
    while current_y >= 0 and not finished:
      if current_x < 0:
        current_x = 0
        finished = True

      if current_x > image.shape[1] - margin:
        current_x = image.shape[1] - margin
        finished = True

      window_roi = np.s_[int(current_y):int(current_y+window_height), int(current_x):int(current_x+margin)]
      
      new_search_window = np.array(np.int32([[[current_x, current_y],
          [current_x + margin, current_y],
          [current_x + margin, current_y + window_height],
          [current_x, current_y + window_height],
          [current_x, current_y]]]),np.int32 )
      
      self._search_windows = np.concatenate((self._search_windows, 
        new_search_window),axis=0)
      # print("x: {} y:{} margin: {}, h: {} search window: {}".format(
      #   current_x, current_y, margin, window_height, self._search_windows.shape))
      window_image = image[window_roi]

      nonzero_this_window = cv2.findNonZero(window_image)
      
      if not nonzero_this_window is None:
        for i in range(nonzero_this_window.shape[0]):
          pt = nonzero_this_window[i]
          pt[0,0] += current_x
          pt[0,1] += current_y
          nonzero_points = np.concatenate([nonzero_points, pt], axis=0)

        if nonzero_this_window.shape[0] > self.lp.minpix and self._do_reset:
          x_pts = cv2.extractChannel(nonzero_this_window, 0)
          mean = x_pts.astype(np.float)
          mean = cv2.reduce(mean, 0, cv2.REDUCE_AVG)
          
          tc = current_x
          current_x = mean[0,0] - margin / 2.0
        
      if not self._do_reset:
        current_x = self._best_fit.CalcX(current_y) - margin / 2.0


      current_y -= window_height
    return nonzero_points


  def FitPoints(self, points):
    if points is None:
      return (False, None)

    length = len(points)
    if length < 3:
      return (False, None)

    A = np.ones((length, 3), np.float)
    b = np.zeros((length,1), np.float)

    for ind in range(length):
      A[ind,0] = points[ind,1] * points[ind,1]
      A[ind,1] = points[ind,1]
      b[ind,0] = points[ind,0]

    x_sys = cv2.solve(A,b, flags=cv2.DECOMP_LU | cv2.DECOMP_NORMAL)
    
    if x_sys[0]:

      m = LaneLineModel()
      m.a = x_sys[1][0]
      m.b = x_sys[1][1]
      m.c = x_sys[1][2]

      return (True, m)
    return False, None

  def compute_curvature_radius(self, model, y):
    my = self.lp.ym_per_pix
    mx = self.lp.xm_per_pix

    Y = y * my
    A = model.a * mx / (my**2)
    B = model.b * mx / my
    Rc = (((2.0 * A*Y + B) ** 2 + 1) ** 1.5) / abs(2.0 * A)
    return Rc[0]

class AdvancedLaneFinder:
  """ a class that has all the machinery needed to process frames"""
  def __init__(self, ap = AdvancedLaneFinderParams(), ip = ImagePipelineParams()):
    self.ap = ap
    self.pipeline = ImagePipeline(ip)

    img_size = self.pipeline.GetFrameSize()
    img_height = img_size[0]
    img_width = img_size[1]
    print("image width: {} image height: {}".format(img_width, img_height))

    self.left_lane = LaneLine(self.ap.lp, np.s_[int(img_height/2):int(img_height),0:int(img_width/2)],'left')
    self.right_lane = LaneLine(self.ap.lp, np.s_[int(img_height/2):int(img_height),int(img_width/2):int(img_width)], 'right')
    

  def Run(self):
    self.SetupWindows()

    self.pipeline.Run(self._ProcessFrame)

  def _ProcessFrame(self, frame):

    undistorted = cv2.undistort(frame, self.ap.K, self.ap.D)
    binary_img = self.threshold_image(undistorted)
    warped_img = cv2.warpPerspective(binary_img, self.ap.M, binary_img.shape[::-1] )
    ret, warped_img = cv2.threshold(warped_img, 128, 255, cv2.THRESH_BINARY)

    self.left_lane.Update(warped_img)
    self.right_lane.Update(warped_img)

    ave_curvature = (self.left_lane.radius_of_curvature_meters + 
      self.right_lane.radius_of_curvature_meters) / 2.0

    lane_center = (self.left_lane.at(warped_img.shape[0]) + self.right_lane.at(warped_img.shape[0]) ) / 2.0

    center_offset = (lane_center[0] - (warped_img.shape[1] / 2.0)) * self.ap.lp.xm_per_pix

    warped_annotations = np.zeros(warped_img.shape + (3,), np.uint8 )

    curve_points = np.concatenate((self.left_lane._best_x, self.right_lane._best_x[::-1]),axis=0)

    if len(curve_points) > 4:
      warped_annotations = cv2.fillPoly(warped_annotations, np.int32([curve_points]), (0,255,0))
    
    annotations = cv2.warpPerspective(warped_annotations, self.ap.M_inv, warped_annotations.shape[0:2][::-1])

    out_frame = cv2.putText(undistorted, "Radius: {:.2f}m".format(ave_curvature),            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3 )
    out_frame = cv2.putText(out_frame, "Lane Center Offset: {:.2f}m".format(center_offset), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3 )

    out_frame = cv2.addWeighted(out_frame, 1.0, annotations, .3, 0)

    if self.ap.show_warped:
      nonzero_channel = np.zeros_like(warped_img, np.uint8)
      nonzero_channel = self.FillPoints(nonzero_channel, self.left_lane.detected_pixels)
      nonzero_channel = self.FillPoints(nonzero_channel, self.right_lane.detected_pixels)

      searchbox_channel = np.zeros_like(warped_img, np.uint8)
      searchbox_channel = cv2.polylines(searchbox_channel, self.left_lane._search_windows, False, (255), 2)
      searchbox_channel = cv2.polylines(searchbox_channel, self.right_lane._search_windows, False, (255), 2)

      searchbox_channel = cv2.polylines(searchbox_channel, np.int32([self.left_lane._best_x]), False, 255, 3)
      searchbox_channel = cv2.polylines(searchbox_channel, np.int32([self.right_lane._best_x]), False, 255, 3)

      warped_final = np.dstack((warped_img, searchbox_channel, nonzero_channel))

      out_frame = cv2.hconcat((out_frame, warped_final))

      empty_chan = np.zeros_like(warped_img, np.uint8)
      bin_img = np.dstack((binary_img, empty_chan, empty_chan))

      empty_frame = np.zeros_like(bin_img)

      out_frame2 = cv2.hconcat((bin_img, empty_frame))

      out_frame = cv2.vconcat((out_frame, out_frame2))
      out_frame = cv2.resize(out_frame, warped_final.shape[0:2][::-1], interpolation=cv2.INTER_LANCZOS4)

    return out_frame
  
  def threshold_image(self, undistorted_image):
    undistorted_gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
    sobel_gray = self.abs_sobelx_threshold(undistorted_gray)

    img_hls = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HLS)
    l_chan = self.threshold_channel(img_hls, 1, self.ap.min_l, self.ap.max_l)
    s_chan = self.threshold_channel(img_hls, 2, self.ap.min_s, self.ap.max_s)
    l_chan_filt = self.threshold_channel(img_hls, 1, 100, 255)

    output_img = cv2.bitwise_or(s_chan, l_chan)
    output_img = cv2.bitwise_and(output_img, l_chan_filt)
    output_img = cv2.bitwise_or(sobel_gray, output_img)

    return output_img
  
  def abs_sobelx_threshold(self, input):
    output = cv2.Sobel(input, cv2.CV_64F,1,0)
    min_v, max_v, min_ind, max_ind = cv2.minMaxLoc(output)
    abs_max = max(abs(min_v), abs(max_v))
    scale = 255.0 / abs_max

    output = cv2.convertScaleAbs(output,alpha=scale)

    output = ThresholdImage(output, self.ap.min_sobel, self.ap.max_sobel)

    return output    

  def threshold_channel(self, img_hls, channel, min_thr, max_thr):
    chan_out = cv2.extractChannel(img_hls, channel)
    return ThresholdImage(chan_out, min_thr, max_thr)

  def FillPoints(self, image, points):
    if points is None:
      return image

    for point in points:
      image[point[1], point[0]] = 255
    return image
  
  def SetupWindows(self):
    if not self.ap.show_trackbars:
      return
    
    cv2.namedWindow(self.ap.trackbar_window, cv2.WINDOW_AUTOSIZE)

    cv2.createTrackbar("sobel_min", self.ap.trackbar_window, self.ap.min_sobel, 255, self.adjust_sobel_min )
    cv2.createTrackbar("sobel_max", self.ap.trackbar_window, self.ap.max_sobel, 255, self.adjust_sobel_max )
    cv2.createTrackbar("min s", self.ap.trackbar_window, self.ap.min_s, 255, self.adjust_s_min )
    cv2.createTrackbar("max s", self.ap.trackbar_window, self.ap.max_s, 255, self.adjust_s_max )
    cv2.createTrackbar("min l", self.ap.trackbar_window, self.ap.min_l, 255, self.adjust_l_min )
    cv2.createTrackbar("max l", self.ap.trackbar_window, self.ap.max_l, 255, self.adjust_l_max )
    cv2.createTrackbar("show warped", self.ap.trackbar_window, self.ap.show_warped, 1, self.adjust_show_warped )

  def adjust_sobel_min(self, value):
    self.ap.min_sobel = value
  
  def adjust_sobel_max(self, value):
    self.ap.max_sobel = value
  
  def adjust_s_min(self, value):
    self.ap.min_s = value

  def adjust_s_max(self, value):
    self.ap.max_s = value

  def adjust_l_min(self, value):
    self.ap.min_l = value

  def adjust_l_max(self, value):
    self.ap.max_l = value

  def adjust_show_warped(self, value):
    self.ap.show_warped = value

