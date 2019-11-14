message(STATUS "configuring chapter 4 exercises")

##################### chapter 4.4 color selection example ######################
add_executable(chap4_color_selection
  color_selection.cpp
  cv_helpers.cpp)

target_link_libraries( chap4_color_selection 
  ${OpenCV_LIBS} 
  fmt::fmt)

##################### chapter 4.6 region masking example #######################
add_executable(chap4_region_masking
  region_masking.cpp
  cv_helpers.cpp)

target_link_libraries( chap4_region_masking 
  ${OpenCV_LIBS} 
  fmt::fmt)

################ chapter 4.13 canny edge detection example #####################
add_executable(chap4_canny_edge
  canny.cpp
  cv_helpers.cpp)

target_link_libraries( chap4_canny_edge
  ${OpenCV_LIBS} 
  fmt::fmt)

#################### chapter 4.15 hough transform example ######################
add_executable(chap4_hough_transform
  hough_transform.cpp
  cv_helpers.cpp)

target_link_libraries( chap4_hough_transform
  ${OpenCV_LIBS} 
  fmt::fmt)

install(TARGETS 
  chap4_color_selection
  chap4_region_masking
  chap4_canny_edge
  chap4_hough_transform
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
