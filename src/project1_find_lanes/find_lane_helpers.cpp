#include "find_lane_helpers.hpp"
#include <opencv2/opencv.hpp>
#include <random>
#include "fmt/format.h"
#include <cassert>

void FindLanes(const cv::Mat& input, cv::Mat& output, const FindLaneParams& p){
  auto y_size = input.size[0];
  auto x_size = input.size[1];

  // convert the image to grayscale
  static cv::Mat gray_image; 
  cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
  
  // Define a kernel size for Gaussian smoothing / blurring and filter the image
  cv::Size kernel_size {p.canny_blur_kernal_size, p.canny_blur_kernal_size};
  static cv::Mat blur_gray; 
  cv::GaussianBlur(gray_image, blur_gray, kernel_size, 0);
  
  // Do the canny edge detection on the image 
  static cv::Mat canny; 
  cv::Canny(blur_gray, canny, p.canny_low, p.canny_high);

  // define the points that will be used to mask the image.  
  std::vector<std::vector<cv::Point>> vertices 
    {{{0,y_size},
    {x_size/2-p.mask_top_width + p.mask_top_offset, (int)(y_size * p.mask_height)},
    {x_size/2+p.mask_top_width + p.mask_top_offset, (int)(y_size * p.mask_height)},
    {x_size,y_size}}};

  //create the actual mask
  static cv::Mat mask;
  mask = cv::Mat::zeros(canny.size(),CV_8UC1);

  cv::fillPoly(mask, vertices, cv::Scalar(255));

  // apply the mask to the canny output. Note that we have to mask the image
  // AFTER doing canny or canny will detect the edges of the mask!
  static cv::Mat masked_canny;
  canny.copyTo(masked_canny, mask);

  // convert the canny back to color so we can annotate in color
  static cv::Mat masked_canny_color;
  cv::cvtColor(masked_canny, masked_canny_color, CV_GRAY2BGR);

  //create blank bgr image to draw lines on
  cv::Mat color_lines = cv::Mat::zeros(masked_canny_color.size(), masked_canny_color.type()); 
  
  // Run Hough on edge detected image
  cv_line_list hough_lines;
  cv::HoughLinesP(masked_canny, hough_lines, p.hough_rho, p.hough_theta, p.hough_threshold, 
    p.hough_min_line_length, p.hough_max_line_gap);
  
  // check the angle each line makes with the y-axis as a first cut at picking 
  // only valid lines 
  cv_line_list good_angle_lines;
  for (const auto& line : hough_lines){
    cv::Point2i p1 {line[0], line[1]}; //x1, y1
    cv::Point2i p2 {line[2], line[3]}; //x2, y2

    // convert points into a vector. Because we don't know order of the points 
    // coming from the hough transform we force the components of the vector to 
    // be positive
    cv::Vec2i line_vector{ std::abs(p2.y - p1.y), std::abs(p2.x - p1.x)}; 
    double length = cv::norm(line_vector);
    
    // from trig, cos(theta) = adjacent / hypotenuse where the adjacent 
    // is just the y component of the hypotenuse.  
    double theta = std::acos(line_vector[0] / length);

    // if the angle is good, send it on
    if( theta > (p.line_min_angle) && theta < (p.line_max_angle) ){
      good_angle_lines.emplace_back(line);

      // this should never happen --if this happens that means theta == 0 which 
      // should've been filtered out from the above checkes. 
      if(p1.y == p2.y ) {
        fmt::format("Error: bad line detected: p1: {}, {} p2: {}, {} theta: {}\n",
          p1.x, p1.y, p2.x, p2.y, theta * kR2D); }}}

  // if we have at least one "good" line lets do ransac
  if(good_angle_lines.size() > 0){

    // create persistent models for each lane line
    static cv::Mat1d left_lane_model(2,1);
    static cv::Mat1d right_lane_model(2,1);
    const double alpha = .7;

    // look for the left lane line doing ransac
    ModelSet left_lane_ransac_out = do_ransac(good_angle_lines, true);
    
    // filter the ransac output with the output from the previous frame
    if(left_lane_ransac_out.inliers.size() > 0){
      left_lane_model = alpha_filter(left_lane_model, left_lane_ransac_out.model, alpha);}
    // and annotate the results on the frame
    plot_model(y_size, left_lane_model, cv::Scalar(0,255,0), &color_lines, &left_lane_ransac_out.inliers);

    // look for the right lane with ransac, filter and annotate the frame
    ModelSet right_lane_ransac_out = do_ransac(good_angle_lines, false);
    if(right_lane_ransac_out.inliers.size() > 0) {
      right_lane_model = alpha_filter(right_lane_model, right_lane_ransac_out.model, alpha);}

    plot_model(y_size, right_lane_model, cv::Scalar(255,0,255), &color_lines, &right_lane_ransac_out.inliers);}
  
  // draw the mask outline
  cv::polylines(color_lines, vertices,true, cv::Scalar(255,0,0),3);
  
  // copy all of the annotated lines onto either the canny output or the
  // original color image and we're done!
  if(p.show_canny){
    cv::addWeighted(color_lines, 0.9, masked_canny_color, 1, 0, output);}
  else{
    cv::addWeighted(color_lines, 0.9, input, 1, 0, output);}} //end FindLanes



ModelSet do_ransac(const cv_line_list& input_lines, bool positive_slope){

  static std::random_device rd;  //Will be used to obtain a seed for the random number engine
  static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

  const size_t num_data_points = input_lines.size();
  std::uniform_int_distribution<> dis(0, input_lines.size() - 1); 
  
  const size_t k = std::min((size_t) 10, num_data_points); // max num iterations
  const double slope_threshold = .2; // slope threshold for a good model
  const double intercept_threshold = 20; // intercept threshold  to determine good model
  
  ssize_t best_model = -1;
  std::map<size_t, ModelSet> models;

  for(size_t iter = 0; iter < k; ++iter){
    size_t model_index = dis(gen);

    if(models.find(model_index) != models.end()) {// we've already done this model
      continue; }
    
    // register the new model
    models[model_index] = ModelSet();
    ModelSet& current_model = models[model_index];
    
    // add the new model to the list of inliers
    current_model.add_inlier(input_lines[model_index]);

    //define the model
    cv::Mat1d b_model(2,1);
    cv::Mat1d A_model(2,2);

    b_model(0,0) = input_lines[model_index][1];
    b_model(1,0) = input_lines[model_index][3];

    A_model(0,0) = input_lines[model_index][0];
    A_model(1,0) = input_lines[model_index][2];
    A_model(0,1) = 1;
    A_model(1,1) = 1;

    // solve for the slope and intercept of the current model
    cv::solve(A_model, b_model, current_model.model);

    // if(positive_slope && current_model.model(0) > 0){
    //   fmt::print("model index {}: {} {} {} {}, m: {} b: {}\n",model_index, 
    //     input_lines[model_index][0], input_lines[model_index][1], 
    //     input_lines[model_index][2], input_lines[model_index][3],
    //     current_model.model(0), current_model.model(1) );
    
    //   assert(in_bounds(current_model.model(1), -1000.0, 1000.0));}

    // assume models with a positive slope are for the left lane and negative
    // for the right lane. check to see if the model matches the requested lane
    if((positive_slope && current_model.model(0) < 0) ||
       (!positive_slope && current_model.model(0) > 0))  {
          continue; }

    cv::Mat1d b_maybe_inlier(2,1);
    cv::Mat1d A_maybe_inlier(2,2);
    cv::Mat1d maybe_inlier_model(2,1);

    // iterate through all the lines to see if each is an inlier or outlier
    // it would probably be better to decompose each line into points and see 
    // how far each point is from the line
    for (size_t ctr = 0; ctr < num_data_points; ++ctr){
      if(ctr == model_index)
        continue;
      
      b_maybe_inlier(0,0) = input_lines[ctr][1];
      b_maybe_inlier(1,0) = input_lines[ctr][3];

      A_maybe_inlier(0,0) = input_lines[ctr][0];
      A_maybe_inlier(1,0) = input_lines[ctr][2];
      A_maybe_inlier(0,1) = 1;
      A_maybe_inlier(1,1) = 1;

      cv::solve(A_maybe_inlier, b_maybe_inlier, maybe_inlier_model);
      cv::Mat1d candidate_error =  current_model.model - maybe_inlier_model;
      
      candidate_error(0) = abs(candidate_error(0));
      candidate_error(1) = abs(candidate_error(1));

      if(candidate_error(0) < slope_threshold &&
         candidate_error(1) < intercept_threshold){
          current_model.add_inlier(input_lines[ctr]);}
      else {
        current_model.outliers.push_back(input_lines[ctr]);}}

    // keep track of the best model from this iteration
    if(best_model < 0) {
      best_model = model_index; }
    else  {
      // the best model has the most inliers
      if(current_model.inliers.size() > models[best_model].inliers.size()){
        best_model = model_index;} } }

  if (best_model < 0){
    fmt::print("No model found\n");
    return {}; }

  //fmt::print("BEST MODEL: {}\n", best_model);
  
  const ModelSet& best_model_set = models[best_model];

  // do a least squares fit os all the points from the inliers
  cv::Mat1d b_best_model(best_model_set.inliers.size(),1);
  cv::Mat1d A_best_model = cv::Mat1d::ones(best_model_set.inliers.size(),2);
  
  int ctr = 0;
  for(const auto& model: best_model_set.inliers){
    b_best_model(ctr,0) = model.y;
    A_best_model(ctr,0) = model.x;
    //fmt::print("  {} {} {}\n", A_inliers(i,0), A_inliers(i,1), y_inliers(i));
    ++ctr; }
  
  cv::solve(A_best_model, b_best_model, best_model_set.model, cv::DECOMP_NORMAL);
  //output.model = x_best;
  //fmt::print("best model {}, inliers: {}\n", best_model, fmt::join(best_model_inliers, ", "));
  return best_model_set;}


void plot_model(int y_size, const cv::Mat1d& model, const cv::Scalar color, 
  cv::Mat* image, std::vector<cv::Point> * points){

  cv::Point2i p1, p2, p3, p4;
  
  double m1 = model(0);
  double b1 = model(1);
  //fmt::print("  m1: {}, b1: {}\n", m1, b1);
  p1.y = y_size/2;
  p1.x = (p1.y - b1)/m1;
  p2.y = y_size;
  p2.x = (p2.y - b1)/m1;
  //fmt::print("    p1.x: {}, p1.y: {}\n", p1.x, p1.y);
  //fmt::print("    p2.x: {}, p2.y: {}\n", p2.x, p2.y);
  cv::line(*image, p1, p2, color, 3, CV_AA);

  for (auto pt : *points){
    cv::drawMarker(*image, pt, color,cv::MARKER_STAR, 10, 4);} }


bool in_bounds(int val, int min, int max){
  return val > min && val < max;}


void adjust_canny_low( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 25, 200)){
    params->canny_low = count;}}

void adjust_canny_high( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 25, 200)){
    params->canny_high = count;}}

void adjust_hough_threshold( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 5, 30)){
    params->hough_threshold = count;}}

void adjust_hough_min_length( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 5, 100)){
    params->hough_min_line_length = count;}}

void adjust_hough_max_gap( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 5, 100)){
    params->hough_max_line_gap = count;}}

void adjust_line_min( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 5, 90)){
    params->line_min_angle = count * kD2R;}}

void adjust_line_max( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 5, 90)){
    params->line_max_angle = count * kD2R;}}

void adjust_top_width( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 0, 100)){
    params->mask_top_width = count;}}

void adjust_top_offset( int count, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  if(in_bounds(count, 0, 50)){
    params->mask_top_offset = count;}}

void toggle_image( int state, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  params->show_canny = state;}

void adjust_frame_speed (int ms_delay, void* param){
  FindLaneParams* params = static_cast<FindLaneParams*>(param);  
  params->frame_delay_ms = ms_delay;}
 
void setup_windows(FindLaneParams* find_lane_params){

  cv::namedWindow("Adjustments",  cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Canny Low","Adjustments", nullptr, 255, &adjust_canny_low, find_lane_params);
  cv::setTrackbarPos("Canny Low","Adjustments",find_lane_params->canny_low);

  cv::createTrackbar("Canny High","Adjustments", nullptr, 255, &adjust_canny_high, find_lane_params);
  cv::setTrackbarPos("Canny High","Adjustments",find_lane_params->canny_high);

  cv::createTrackbar("Hough threshold","Adjustments", nullptr, 30, &adjust_hough_threshold, find_lane_params);
  cv::setTrackbarPos("Hough threshold","Adjustments",find_lane_params->hough_threshold);

  cv::createTrackbar("Hough min len","Adjustments", nullptr, 100, &adjust_hough_min_length, find_lane_params);
  cv::setTrackbarPos("Hough min len","Adjustments",find_lane_params->hough_min_line_length);

  cv::createTrackbar("Hough max gap","Adjustments", nullptr, 100, &adjust_hough_max_gap, find_lane_params);
  cv::setTrackbarPos("Hough max gap","Adjustments",find_lane_params->hough_max_line_gap);

  cv::createTrackbar("line angle min","Adjustments", nullptr, 100, &adjust_line_min, find_lane_params);
  cv::setTrackbarPos("line angle min","Adjustments",find_lane_params->line_min_angle * kR2D);

  cv::createTrackbar("line angle max","Adjustments", nullptr, 100, &adjust_line_max, find_lane_params);
  cv::setTrackbarPos("line angle max","Adjustments",find_lane_params->line_max_angle * kR2D);

  cv::createTrackbar("mask top width","Adjustments", nullptr, 100, &adjust_top_width, find_lane_params);
  cv::setTrackbarPos("mask top width","Adjustments",find_lane_params->mask_top_width);

  cv::createTrackbar("mask top offset","Adjustments", nullptr, 100, &adjust_top_offset, find_lane_params);
  cv::setTrackbarPos("mask top offset","Adjustments",find_lane_params->mask_top_offset);

  cv::createTrackbar("Show canny image","Adjustments", nullptr, 1, &toggle_image, find_lane_params);
  cv::setTrackbarPos("Show canny image","Adjustments",find_lane_params->show_canny);
  
  cv::createTrackbar("Frame delay","Adjustments", nullptr, 200, &adjust_frame_speed, find_lane_params);
  cv::setTrackbarPos("Frame delay","Adjustments",find_lane_params->frame_delay_ms);}



