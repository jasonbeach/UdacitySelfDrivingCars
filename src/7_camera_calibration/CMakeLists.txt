message(STATUS "configuring chapter 7 exercises")

##################### chapter 7.10 checkerboard example ######################
add_executable(c7_checkerboard
  checkerboard.cpp
  calibrate.cpp)

target_link_libraries(c7_checkerboard
  jbeach::cv_common 
  fmt::fmt
  CLI11::CLI11)

install(TARGETS 
  c7_checkerboard
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
