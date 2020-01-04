#include "AdvancedLaneFinder.hpp"

// unfortunately opencv trackbars accept only c-style function pointers
// so we have to define these outside the class to work.
static void adjust_sobel_min( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, 0, p->max_sobel)){
    p->min_sobel = count;}}

static void adjust_sobel_max( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, p->min_sobel, 255)){
    p->max_sobel = count;}}

static void adjust_s_min( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, 0, p->max_s)){
    p->min_s = count;}}

static void adjust_s_max( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, p->min_s, 255)){
    p->max_s = count;}}

static void adjust_l_min( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, 0, p->max_l)){
    p->min_l = count;}}

static void adjust_l_max( int count, void* param){
  auto p = static_cast<AdvancedLaneFinderParams*>(param);
  
  if(in_bounds<uint8_t>(count, p->min_l, 255)){
    p->max_l = count;}}

class AdvancedLaneFinder::AdvancedLaneFinderImpl{
  public:
    AdvancedLaneFinderImpl(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip):
      ap_(ap), pipeline_(ip) {}

    void Run(){
      SetupWindows();

      pipeline_.Run([this](const cv::Mat img){
        cv::Mat processed_image;
        ProcessFrame(img, processed_image);
        return processed_image; } ); }

  private:
    void ProcessFrame(const cv::Mat& frame, cv::Mat& out_frame){
      cv::Mat undistorted;
      cv::undistort(frame, undistorted, ap_.K, ap_.D);
      
      cv::Mat undistorted_gray;
      cv::cvtColor(undistorted, undistorted_gray, cv::COLOR_BGR2GRAY);

      cv::Mat sobel_gray;
      abs_sobelx_threshold(undistorted_gray, sobel_gray, ap_.min_sobel, ap_.max_sobel);

      cv::Mat img_hls;
      cv::cvtColor(undistorted, img_hls, cv::COLOR_BGR2HLS);

      cv::Mat l_chan_filt, s_chan, l_chan;
      threshold_channel(img_hls, l_chan, 1, ap_.min_l, ap_.max_l);
      threshold_channel(img_hls, s_chan, 2, ap_.min_s, ap_.max_s);
      threshold_channel(img_hls, l_chan_filt, 1, 100, 255);

      cv::Mat binary_img;
      cv::bitwise_or(s_chan, l_chan, binary_img);
      cv::bitwise_and(binary_img, l_chan_filt, binary_img);
      cv::bitwise_or(sobel_gray, binary_img, binary_img);

      cv::Mat warped_img;
      cv::warpPerspective(binary_img, warped_img, ap_.M, binary_img.size());
      cv::threshold(warped_img, warped_img, 128, 255, CV_THRESH_BINARY);

      int left_lane_starting_point = get_max_col(warped_img, cv::Rect(0,warped_img.rows/2,warped_img.cols/2, warped_img.rows/2));
      std::vector<cv::Point> left_lane_points = get_lane_line_points(warped_img, left_lane_starting_point);
      LaneLineModel left_model_pix = FitPoints(left_lane_points);
      float left_lane_curvature = compute_curvature_radius(left_model_pix, ap_.xm_per_pix, ap_.ym_per_pix, 720);

      int right_lane_starting_point = get_max_col(warped_img, cv::Rect(warped_img.cols/2, warped_img.rows/2, warped_img.cols/2, warped_img.rows/2));
      std::vector<cv::Point> right_lane_points = get_lane_line_points(warped_img, right_lane_starting_point);
      LaneLineModel right_model_pix = FitPoints(right_lane_points);
      float right_lane_curvature = compute_curvature_radius(right_model_pix, ap_.xm_per_pix, ap_.ym_per_pix, 720);

      float ave_curvature = (left_lane_curvature + right_lane_curvature) / 2.0;

      float lane_center = (left_model_pix.calc_x(warped_img.rows) + right_model_pix.calc_x(warped_img.rows)) / 2.0;
      float center_offset = (lane_center - (warped_img.cols /2.0)) * ap_.xm_per_pix;

      cv::Mat annotations = cv::Mat::zeros(undistorted.size(), CV_8U);
      cv::putText(annotations, fmt::format("Radius:{:.2f}m", ave_curvature), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {255}, 3);
      cv::putText(annotations, fmt::format("Lane Center Offset: {:.2f}m", center_offset), {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 1, {255}, 3 );

      out_frame = undistorted.clone();
      out_frame.setTo(cv::Scalar(255,0,0), binary_img);
      out_frame.setTo(cv::Scalar(0,255,0), annotations);

      if(ap_.show_warped){
        //paint nonzero points
        cv::Mat nonzero_channel = cv::Mat::zeros(warped_img.size(), CV_8U); 
        FillPoints(nonzero_channel, left_lane_points);
        FillPoints(nonzero_channel, right_lane_points);
        //paint fit curves
        DrawCurve(annotations, left_model_pix);
        DrawCurve(annotations, right_model_pix);
        // combine channels
        cv::Mat warped_final;
        cv::Mat arry[3] = {warped_img, annotations, nonzero_channel};
        cv::merge(arry, 3, warped_final); 
        cv::imshow("warped", warped_final);} }

    void abs_sobelx_threshold(const cv::Mat& input, cv::Mat& output, uint8_t min_sobel, uint8_t max_sobel){

      cv::Sobel(input, output, CV_64F, 1, 0);

      double abs_max = AbsMax<double>(output);
      double scale = 255.0 / abs_max;

      cv::convertScaleAbs(output, output, scale);

      ThresholdImage(output, output, min_sobel, max_sobel);}



    void threshold_channel(const cv::Mat& img_hls, cv::Mat& chan_out, uint8_t channel, uint8_t min_thresh, uint8_t max_thresh){
      cv::extractChannel(img_hls, chan_out, channel);
      ThresholdImage(chan_out, chan_out, min_thresh, max_thresh); }

    int get_max_col(const cv::Mat& image, const cv::Rect& roi){
      
      cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U); // all 0
      mask(roi) = 255;
      
      //mask input image
      cv::Mat masked_image;
      image.copyTo(masked_image, mask);
  
      cv::Mat summed;
      cv::reduce(masked_image, summed, 0, CV_REDUCE_SUM, CV_32S);

      cv::Point max_col;

      cv::minMaxLoc(summed, nullptr, nullptr, nullptr, &max_col);
      return max_col.x;}

    std::vector<cv::Point> get_lane_line_points(const cv::Mat& image, int start_col){

      // calc window height
      int window_height = image.rows / ap_.nwindows;

      //init current_x and current_y
      int current_x = start_col - ap_.margin / 2; //upper left corner of rect
      int current_y = image.rows - window_height;            // upper left corner of rect
      
      //cv::Mat empty = cv::Mat::zeros(image.size(), CV_8U);
      //cv::Mat nonzero_channel = cv::Mat::zeros(image.size(), CV_8U);
      std::vector<cv::Point> nonzero_points;
      bool finished = false;
      while(current_y >= 0 && !finished){

        if(current_x < 0){
          current_x = 0; 
          finished = true;}

        if(current_x > image.cols - ap_.margin){
          current_x = image.cols - ap_.margin;
          finished = true;}

        cv::Rect window_roi = cv::Rect(current_x, current_y, ap_.margin, window_height);
        cv::Mat window_image = image(window_roi);

        cv::Mat nonzero_this_window;
        cv::findNonZero(window_image, nonzero_this_window);

        for(int i = 0; i < nonzero_this_window.rows; ++i){
          cv::Point pt = nonzero_this_window.at<cv::Point>(i);
          pt.x += current_x;
          pt.y += current_y;
          nonzero_points.push_back(pt);}

        if(nonzero_this_window.rows > ap_.minpix){
          cv::Mat x_pts;
          cv::extractChannel(nonzero_this_window, x_pts, 0);
          cv::Mat mean;
          x_pts.convertTo(mean, CV_32F);
          cv::reduce(mean, mean, 0, CV_REDUCE_AVG, CV_32F);
          current_x += mean.at<float>(0,0) - ap_.margin / 2.0; }
        
        current_y -= window_height; }

      return nonzero_points;}

    LaneLineModel FitPoints(const std::vector<cv::Point> points){
      
      LaneLineModel model_pix;

      auto length = points.size();
      if(length < 3){
        return model_pix; }
      
      cv::Mat A = cv::Mat::ones(length, 3, CV_32F);
      cv::Mat b(length, 1, CV_32F);

      //use 'y' as the independent variable
      for(size_t i = 0; i < points.size(); ++i){
        A.at<float>(i, 0) = points[i].y * points[i].y;
        A.at<float>(i, 1) = points[i].y;
        b.at<float>(i, 0) = points[i].x; }

      //fmt::print("A: {}, b: {}\n", A, b);

      cv::Mat x_sys;
      try{
        cv::solve(A, b, x_sys, cv::DECOMP_LU | cv::DECOMP_NORMAL);}
      catch(const cv::Exception& e) {
        fmt::print("Caught exception: {} points length: {}\n", e.what(), length);}

      model_pix.a = x_sys.at<float>(0,0);
      model_pix.b = x_sys.at<float>(1,0);
      model_pix.c = x_sys.at<float>(2,0);
      model_pix.valid = true;

      return model_pix; }

    // from https://knowledge.udacity.com/questions/29265
    // we just solved 
    //         x = a*y^2 + b*y + c    eq(1) 
    // where a, b, c, x and y are in pixel space
    // we want 
    //         X = A*Y^2 + B*Y + C   eq(2) 
    // where A, B, C, X and Y are in real space meter)
    // let X = x*mx and Y = y*my
    // plugging these into eq 2: 
    //         x*mx = A*y^2*my^2 + B*y*my + C  eq(3)
    // or 
    //             A*y^2*my^2 + B*y*my + C   
    //         x = ----------   ------   -     eq(4)
    //                 mx         mx     mx
    //
    // by comparing similar terms between eq(1) and eq(4) we can infer that
    //             a * mx        b*mx       
    //         A = ------   B = -------  C = c*mx   eq(5)
    //              my^2          my
    //
    // radius of curvature in pixel space is:
    //
    //             (1+(2*a*y+b)^2)^1.5
    //        rc = -------------------        eq(6)
    //                    |2*a|
    //
    // or in real world coordinates
    //
    //             (1+(2*A*Y+B)^2)^1.5
    //        Rc = -------------------        eq(6)
    //                    |2*A|

    float compute_curvature_radius(const LaneLineModel& model, float mx, float my, float y){
      float Y = y * my;
      float A = model.a * mx / SQ(my);
      float B = model.b * mx / my;
      float Rc = std::pow(SQ(2.0*A*Y + B)+1, 1.5) / std::abs(2.0*A);
      return Rc;}
    
    void FillPoints(cv::Mat& image, const std::vector<cv::Point> points){
      for(const auto& point : points){
        image.at<uint8_t>(point.y, point.x) = 255; } }

    void DrawCurve(cv::Mat& image, const LaneLineModel& m){
      std::vector<cv::Point> poly;
      for (int y = 0; y < image.rows; y+=20){
        float x = m.calc_x(y);
        if(x >=0 && x <= image.cols){
          poly.emplace_back(x , y  );}}

      cv::polylines(image, poly, false, {255}, 3);}

    void SetupWindows(){
    
      if(! ap_.show_trackbars){
        return; }

      cv::namedWindow(ap_.trackbar_window, CV_WINDOW_AUTOSIZE);
 
      cv::createTrackbar("sobel min", ap_.trackbar_window, nullptr, 255, &adjust_sobel_min, &ap_);
      cv::setTrackbarPos("sobel min", ap_.trackbar_window, ap_.min_sobel);
    
      cv::createTrackbar("sobel max", ap_.trackbar_window, nullptr, 255, &adjust_sobel_max, &ap_);
      cv::setTrackbarPos("sobel max", ap_.trackbar_window, ap_.max_sobel);

      cv::createTrackbar("min s", ap_.trackbar_window, nullptr, 255, &adjust_s_min, &ap_);
      cv::setTrackbarPos("min s", ap_.trackbar_window, ap_.min_s);
    
      cv::createTrackbar("max s", ap_.trackbar_window, nullptr, 255, &adjust_s_max, &ap_);
      cv::setTrackbarPos("max s", ap_.trackbar_window, ap_.max_s);

      cv::createTrackbar("min l", ap_.trackbar_window, nullptr, 255, &adjust_l_min, &ap_);
      cv::setTrackbarPos("min l", ap_.trackbar_window, ap_.min_l);
    
      cv::createTrackbar("max l", ap_.trackbar_window, nullptr, 255, &adjust_l_max, &ap_);
      cv::setTrackbarPos("max l", ap_.trackbar_window, ap_.max_l);

      cv::createTrackbar("show warped", ap_.trackbar_window, &ap_.show_warped, 1);}

    AdvancedLaneFinderParams ap_;
    ImagePipeline pipeline_;
};

AdvancedLaneFinder::AdvancedLaneFinder(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip): 
  impl_{std::make_unique<AdvancedLaneFinderImpl>(ap, ip)} {}

AdvancedLaneFinder::AdvancedLaneFinder(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder& AdvancedLaneFinder::operator=(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder::~AdvancedLaneFinder() = default;

void AdvancedLaneFinder::Run(){ Impl()->Run();}