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


using PointVector = std::vector<cv::Point>;
using PointVectorList = std::vector<PointVector>;

struct LaneLineModel{
  float a = 0;
  float b = 0;
  float c = 0;
  bool valid = false;
  
  float CalcX(float y) const{
    return a*SQ(y) + b*y + c; } 

  std::string str(){
    return fmt::format("a: {:.4f}, b: {:.4f}, c: {:.4f}", a, b, c);
  }

  PointVector CurvePoints(int y_min, int y_max, int x_min, int x_max, int step = 20) const{
    PointVector curve;
    for (int y = y_min; y < y_max; y+=step){
      float x = CalcX(y);
      if(x >= x_min && x <= x_max){
        curve.emplace_back(x , y ); }}
    return curve; } };


// ******************* Lane Line class ***************************************
class LaneLine{
  public:
    LaneLine() = default;
    LaneLine(const LaneLineParams& lp, const cv::Rect& starting_roi): lp_(lp), roi_(starting_roi) {}
    void SetConfig(const LaneLineParams& lp, const cv::Rect& starting_roi){
      lp_ = lp;
      roi_ = starting_roi; }

    void Update(const cv::Mat& warped_img){
      
      detected_pixels_ = find_points(warped_img);

      if(FitPoints(detected_pixels_, current_fit_)){
        float current_curvature = compute_curvature_radius(current_fit_, 720);
        radius_of_curvature_m_ = alpha_filter(radius_of_curvature_m, current_curvature, .7);

        float alpha = .7;
        best_fit_.a = alpha_filter(best_fit_.a, current_fit_.a, alpha);
        best_fit_.b = alpha_filter(best_fit_.b, current_fit_.b, alpha);
        best_fit_.c = alpha_filter(best_fit_.c, current_fit_.c, alpha); 
      
        best_x_ = best_fit_.CurvePoints(0, warped_img.rows, 0, warped_img.cols);

        num_good_fits++;

        if(num_good_fits > 5){
          missed_detects_ = 0;
          do_reset_ = false;} }
      else  {
        missed_detects_++;
        if(missed_detects_ > 10){
          fmt::print("failed to find lane line: {}\n", missed_detects_);
          num_good_fits = 0;
          do_reset_ = true;}}}

    int at(int y){
      return best_fit_.CalcX(y);}

    const float& radius_of_curvature_m = radius_of_curvature_m_;
    const PointVector& best_x = best_x_;
    const PointVector& detected_pixels = detected_pixels_;
    const PointVectorList& search_windows = search_windows_;

  private:
    LaneLineParams lp_;
    cv::Rect roi_;
    int missed_detects_ = 0;
    int num_good_fits = 0;
    bool detected = false; // was the line detected in the last iteration?
    bool do_reset_ = true;


    PointVectorList recent_xfitted; // x values of the last n fits of the line
    
    PointVector best_x_; // average x values of the fitted line over the last n iterations     
          
    LaneLineModel best_fit_; // polynomial coefficients averaged over the last n iterations

    LaneLineModel current_fit_; // polynomial coefficients for the most recent fit

    float radius_of_curvature_m_ = 0; // radius of curvature of the line in meters

    PointVectorList search_windows_; //windows where we looked for pixels

    //distance in meters of vehicle center from the line
    // self.line_base_pos = None 

    // difference in fit coefficients between last and new fits
    // self.diffs = np.array([0,0,0], dtype='float') 

    PointVector detected_pixels_;  // values for detected pixels
    
    int get_max_col(const cv::Mat& image){
      
      cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U); // all 0
      mask(roi_) = 255;
      
      //mask input image
      cv::Mat masked_image;
      image.copyTo(masked_image, mask);
  
      cv::Mat summed;
      cv::reduce(masked_image, summed, 0, CV_REDUCE_SUM, CV_32S);

      cv::Point max_col;

      cv::minMaxLoc(summed, nullptr, nullptr, nullptr, &max_col);
      return max_col.x;}

    PointVector find_points(const cv::Mat& image){

      int start_col = 0;
      int margin = 0;
      if(do_reset_){
        start_col = get_max_col(image);
        margin = lp_.margin * 2.0; } //find starting location via histogram
      else{
        start_col = best_fit_.CalcX(image.rows);
        margin = lp_.margin;} //find via current model

      // calc window height
      int window_height = image.rows / lp_.nwindows;

      //init current_x and current_y
      int current_x = start_col - margin / 2; //upper left corner of rect
      int current_y = image.rows - window_height;            // upper left corner of rect
      
      search_windows_.clear();

      PointVector nonzero_points;
      bool finished = false;
      while(current_y >= 0 && !finished){

        if(current_x < 0){
          current_x = 0; 
          finished = true;}

        if(current_x > image.cols - margin){
          current_x = image.cols - margin;
          finished = true;}

        

        cv::Rect window_roi = cv::Rect(current_x, current_y, margin, window_height);
        search_windows_.push_back(PointVector(
          {{current_x, current_y},
          {current_x + margin, current_y},
          {current_x + margin, current_y + window_height},
          {current_x, current_y + window_height},
          {current_x, current_y}} ));

        cv::Mat window_image = image(window_roi);

        cv::Mat nonzero_this_window;
        cv::findNonZero(window_image, nonzero_this_window);

        for(int i = 0; i < nonzero_this_window.rows; ++i){
          cv::Point pt = nonzero_this_window.at<cv::Point>(i);
          pt.x += current_x;
          pt.y += current_y;
          nonzero_points.push_back(pt);}

        if(nonzero_this_window.rows > lp_.minpix && do_reset_){
          cv::Mat x_pts;
          cv::extractChannel(nonzero_this_window, x_pts, 0);
          cv::Mat mean;
          x_pts.convertTo(mean, CV_32F);
          cv::reduce(mean, mean, 0, CV_REDUCE_AVG, CV_32F);
          current_x += mean.at<float>(0,0) - margin / 2.0; }

        if(!do_reset_){
          current_x = best_fit_.CalcX(current_y) - margin / 2.0; }

        current_y -= window_height; }

      return nonzero_points;}

    bool FitPoints(const PointVector points, LaneLineModel& model_out){
      
      auto length = points.size();
      if(length < 3){
        return false; }
      
      cv::Mat A = cv::Mat::ones(length, 3, CV_32F);
      cv::Mat b(length, 1, CV_32F);

      //use 'y' as the independent variable
      for(size_t i = 0; i < points.size(); ++i){
        A.at<float>(i, 0) = points[i].y * points[i].y;
        A.at<float>(i, 1) = points[i].y;
        b.at<float>(i, 0) = points[i].x; }

      //fmt::print("A: {}, b: {}\n", A, b);

      cv::Mat x_sys;
      cv::solve(A, b, x_sys, cv::DECOMP_LU | cv::DECOMP_NORMAL);
      

      model_out.a = x_sys.at<float>(0,0);
      model_out.b = x_sys.at<float>(1,0);
      model_out.c = x_sys.at<float>(2,0);

      return true; }

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

    float compute_curvature_radius(const LaneLineModel& model, float y){
      float my = lp_.ym_per_pix;
      float mx = lp_.xm_per_pix;

      float Y = y * my;
      float A = model.a * mx / SQ(my);
      float B = model.b * mx / my;
      float Rc = std::pow(SQ(2.0*A*Y + B)+1, 1.5) / std::abs(2.0*A);
      return Rc;}
};

class AdvancedLaneFinder::AdvancedLaneFinderImpl{
  public:
    AdvancedLaneFinderImpl(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip):
      ap_(ap), pipeline_(ip) {
      
      cv::Size img_size = pipeline_.GetFrameSize();

      left_line_.SetConfig(ap_.lp, cv::Rect(0, img_size.height / 2, img_size.width / 2, img_size.height / 2)); 
      right_line_.SetConfig(ap_.lp, cv::Rect(img_size.width / 2, img_size.height / 2, img_size.width / 2, img_size.height / 2)); }

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
      
      cv::Mat binary_img;
      threshold_image(undistorted, binary_img);

      cv::Mat warped_img;
      cv::warpPerspective(binary_img, warped_img, ap_.M, binary_img.size());
      cv::threshold(warped_img, warped_img, 128, 255, CV_THRESH_BINARY);

      left_line_.Update(warped_img);
      right_line_.Update(warped_img);  


      float ave_curvature = (left_line_.radius_of_curvature_m + right_line_.radius_of_curvature_m) / 2.0;

      float lane_center = (left_line_.at(warped_img.rows) + right_line_.at(warped_img.rows)) / 2.0;
      float center_offset = (lane_center - (warped_img.cols /2.0)) * ap_.lp.xm_per_pix;



      cv::Mat warped_annotations = cv::Mat::zeros(warped_img.size(), CV_8UC3);
      PointVectorList curve_points;

      // fillPoly can handle multiple polys and so wants a vector of polys or a 
      // vector of vector of points
      curve_points.push_back(left_line_.best_x);
      curve_points.at(0).insert(curve_points.at(0).end(), right_line_.best_x.rbegin(), right_line_.best_x.rend());

      
      if(curve_points.at(0).size() > 4){
        cv::fillPoly(warped_annotations, curve_points, cv::Scalar(0,255,0));}

      cv::Mat annotations;// = cv::Mat::zeros(undistorted.size(), CV_8U);
      cv::warpPerspective(warped_annotations, annotations, ap_.M_inv, warped_annotations.size());
      
      //cv::imshow("unwarped poly", annotations);

      out_frame = undistorted.clone();

      cv::putText(out_frame, fmt::format("Radius:{:.2f}m", ave_curvature), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0}, 3);
      cv::putText(out_frame, fmt::format("Lane Center Offset: {:.2f}m", center_offset), {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0}, 3 );

      cv::addWeighted(out_frame, 1, annotations, .3, 0, out_frame);

      //out_frame.setTo(cv::Scalar(255,0,0), binary_img);
      //out_frame.setTo(cv::Scalar(0,255,0), annotations);

      if(ap_.show_warped){
        //paint nonzero points
        cv::Mat nonzero_channel = cv::Mat::zeros(warped_img.size(), CV_8U); 
        FillPoints(nonzero_channel, left_line_.detected_pixels);
        FillPoints(nonzero_channel, right_line_.detected_pixels);

        cv::Mat searchbox_channel = cv::Mat::zeros(warped_img.size(), CV_8U);
        cv::polylines(searchbox_channel, left_line_.search_windows, false, {255},2);
        cv::polylines(searchbox_channel, right_line_.search_windows, false, {255},2);
        
        //paint fit curves
        cv::polylines(searchbox_channel, left_line_.best_x, false, {255},3);
        cv::polylines(searchbox_channel, right_line_.best_x, false, {255},3);

        // combine channels
        cv::Mat warped_final;
        cv::Mat arry[3] = {warped_img, searchbox_channel, nonzero_channel};
        cv::merge(arry, 3, warped_final);
        cv::hconcat(out_frame, warped_final, out_frame);

        cv::Mat empty_chan = cv::Mat::zeros(warped_img.size(), CV_8U);
        cv::Mat arry2[3] = {binary_img, empty_chan, empty_chan};
        cv::Mat temp;
        cv::merge(arry2, 3, warped_final);
        arry2[0] = empty_chan;
        cv::merge(arry2, 3, temp);
        cv::hconcat(warped_final, temp,warped_final);
        cv::vconcat(out_frame, warped_final, out_frame);
        cv::resize(out_frame, out_frame, binary_img.size(),0,0, cv::INTER_LANCZOS4);

        /*cv::imshow("warped", warped_final);*/} }
    
    void threshold_image(const cv::Mat& undistorted_img, cv::Mat& output_img){
      cv::Mat undistorted_gray;
      cv::cvtColor(undistorted_img, undistorted_gray, cv::COLOR_BGR2GRAY);

      cv::Mat sobel_gray;
      abs_sobelx_threshold(undistorted_gray, sobel_gray, ap_.min_sobel, ap_.max_sobel);

      cv::Mat img_hls;
      cv::cvtColor(undistorted_img, img_hls, cv::COLOR_BGR2HLS);

      cv::Mat l_chan_filt, s_chan, l_chan;
      threshold_channel(img_hls, l_chan, 1, ap_.min_l, ap_.max_l);
      threshold_channel(img_hls, s_chan, 2, ap_.min_s, ap_.max_s);
      threshold_channel(img_hls, l_chan_filt, 1, 100, 255);

      cv::bitwise_or(s_chan, l_chan, output_img);
      cv::bitwise_and(output_img, l_chan_filt, output_img);
      cv::bitwise_or(sobel_gray, output_img, output_img); }
    
    void abs_sobelx_threshold(const cv::Mat& input, cv::Mat& output, uint8_t min_sobel, uint8_t max_sobel){

      cv::Sobel(input, output, CV_64F, 1, 0);

      double abs_max = AbsMax<double>(output);
      double scale = 255.0 / abs_max;

      cv::convertScaleAbs(output, output, scale);

      ThresholdImage(output, output, min_sobel, max_sobel);}



    void threshold_channel(const cv::Mat& img_hls, cv::Mat& chan_out, uint8_t channel, uint8_t min_thresh, uint8_t max_thresh){
      cv::extractChannel(img_hls, chan_out, channel);
      ThresholdImage(chan_out, chan_out, min_thresh, max_thresh); }

    void FillPoints(cv::Mat& image, const PointVector points){
      for(const auto& point : points){
        image.at<uint8_t>(point.y, point.x) = 255; } }

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
    LaneLine left_line_;
    LaneLine right_line_;};

AdvancedLaneFinder::AdvancedLaneFinder(const AdvancedLaneFinderParams& ap, const ImagePipelineParams& ip): 
  impl_{std::make_unique<AdvancedLaneFinderImpl>(ap, ip)} {}

AdvancedLaneFinder::AdvancedLaneFinder(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder& AdvancedLaneFinder::operator=(AdvancedLaneFinder&&) = default;

AdvancedLaneFinder::~AdvancedLaneFinder() = default;

void AdvancedLaneFinder::Run(){ Impl()->Run();}