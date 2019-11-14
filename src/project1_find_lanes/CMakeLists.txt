message(STATUS "configuring project 1 find lanes")

##################### chapter 4 Lane Finding Project ###########################
add_executable(FindLaneProject
  find_lane_project.cpp
  find_lane_helpers.cpp)

target_link_libraries( FindLaneProject
  ${OpenCV_LIBS} 
  fmt::fmt
  CLI11::CLI11)



install(TARGETS 
  FindLaneProject
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)