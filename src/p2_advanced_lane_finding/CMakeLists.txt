##################### project 2 advanced lane finding ######################
add_executable(p2_advanced_lane_finding
  p2_main.cpp
  AdvancedLaneFinder.cpp
  config.cpp)

target_link_libraries(p2_advanced_lane_finding
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

install(TARGETS 
  p2_advanced_lane_finding
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES
  project2_config.yaml
  DESTINATION ${CMAKE_INSTALL_BINDIR})
