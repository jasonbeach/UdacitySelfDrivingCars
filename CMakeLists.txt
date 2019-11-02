cmake_minimum_required(VERSION 3.14)
project(udacity VERSION 0.1.0)

include(GNUInstallDirs)
include(FetchContent)
#include(CTest)
#enable_testing()
set(CMAKE_INSTALL_MESSAGE LAZY) # suppress some install messages
set(CMAKE_CXX_STANDARD 17) # use c++ 17
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions

find_package(OpenCV REQUIRED )

FetchContent_Declare(
  fmt_fc
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        6.0.0
)

FetchContent_Declare(
  cli11_fc
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v1.8.0
)

FetchContent_MakeAvailable(fmt_fc cli11_fc)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install CACHE PATH "Installation dir" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
