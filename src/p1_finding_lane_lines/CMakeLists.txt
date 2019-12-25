message(STATUS "configuring project 1 finding lane lines")

##################### Project 1 - Finding Lane Lines ###########################
add_executable(p1_finding_lane_lines
  main.cpp
  finding_lane_lines.cpp)

target_link_libraries(p1_finding_lane_lines
  jbeach::cv_common 
  fmt::fmt
  CLI11::CLI11)

install(TARGETS 
  p1_finding_lane_lines
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)