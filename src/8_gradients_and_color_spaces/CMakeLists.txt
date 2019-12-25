##################### c8 sobel operator ######################
add_executable(c8_sobel
  sobel_main.cpp
  threshold_functions.cpp)

target_link_libraries(c8_sobel
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

##################### c8 gradient mag ######################
add_executable(c8_gradient_mag
  mag_main.cpp
  threshold_functions.cpp)

target_link_libraries(c8_gradient_mag
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

##################### c8 gradient dir ######################
add_executable(c8_gradient_dir
dir_main.cpp
threshold_functions.cpp)

target_link_libraries(c8_gradient_dir
jbeach::cv_common
fmt::fmt
CLI11::CLI11
yaml-cpp)

##################### c8 sobel combined ######################
add_executable(c8_sobel_combined
combined_main.cpp
threshold_functions.cpp)

target_link_libraries(c8_sobel_combined
jbeach::cv_common
fmt::fmt
CLI11::CLI11
yaml-cpp)


##################### c8 colors ######################
add_executable(c8_colors
  color_main.cpp
  threshold_functions.cpp)

target_link_libraries(c8_colors
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

install(TARGETS 
  c8_sobel
  c8_gradient_mag
  c8_gradient_dir
  c8_sobel_combined
  c8_colors
  
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
  config.yaml
  DESTINATION ${CMAKE_INSTALL_BINDIR})
