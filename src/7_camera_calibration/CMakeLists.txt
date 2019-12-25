message(STATUS "configuring chapter 7 exercises")

##################### chapter 7.10 checkerboard example ######################
add_executable(c7_checkerboard
  checkerboard.cpp
  calibrate.cpp)

target_link_libraries(c7_checkerboard
  jbeach::cv_common 
  fmt::fmt
  CLI11::CLI11)

##################### chapter 7.12 distortion correction ######################
add_executable(c7_calibrate_camera
  calibrate_camera.cpp
  calibrate.cpp)

target_link_libraries(c7_calibrate_camera
  jbeach::cv_common 
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

##################### c7 perspective transform ######################
add_executable(c7_perspective_transform
  perspective.cpp)

target_link_libraries(c7_perspective_transform
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

install(TARGETS 
  c7_checkerboard
  c7_calibrate_camera
  c7_perspective_transform
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
  config.yaml
  DESTINATION ${CMAKE_INSTALL_BINDIR})
