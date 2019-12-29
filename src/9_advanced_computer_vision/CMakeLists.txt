##################### c9 warped perspective ######################
add_executable(c9_warped
  warped_main.cpp
  threshold_functions.cpp)

target_link_libraries(c9_warped
  jbeach::cv_common
  fmt::fmt
  CLI11::CLI11
  yaml-cpp)

install(TARGETS 
  c9_warped
  
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
  config.yaml
  DESTINATION ${CMAKE_INSTALL_BINDIR})
