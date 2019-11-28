message(STATUS "configuring chapter 5 exercises")

##################### chapter 5.4 color selection example ######################
add_executable(c5_color_selection
  color_selection.cpp)

target_link_libraries(c5_color_selection 
  jbeach::cv_common 
  fmt::fmt)

##################### chapter 5.6 region masking example #######################
add_executable(c5_region_masking
  region_masking.cpp)

target_link_libraries(c5_region_masking 
  jbeach::cv_common 
  fmt::fmt)

################ chapter 5.13 canny edge detection example #####################
add_executable(c5_canny_edge
  canny.cpp)

target_link_libraries(c5_canny_edge
  jbeach::cv_common 
  fmt::fmt)

#################### chapter 5.15 hough tra  5nsform example ######################
add_executable(c5_hough_transform
  hough_transform.cpp)

target_link_libraries(c5_hough_transform
  jbeach::cv_common 
  fmt::fmt)

install(TARGETS 
  c5_color_selection
  c5_region_masking
  c5_canny_edge
  c5_hough_transform
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
