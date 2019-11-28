message(STATUS "configuring common utilitiles")

add_library(cv_common
  cv_common.cpp
  )

target_include_directories(cv_common
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  )

target_link_libraries(cv_common 
  PUBLIC
  ${OpenCV_LIBS})

add_library(jbeach::cv_common ALIAS cv_common)