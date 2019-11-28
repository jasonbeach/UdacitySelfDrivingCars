cmake_minimum_required(VERSION 3.14)
project(udacity VERSION 0.1.0)

include(GNUInstallDirs)
include(FetchContent)
#include(CTest)
#enable_testing()
set(CMAKE_INSTALL_MESSAGE LAZY) # suppress some install messages

set(CMAKE_CXX_STANDARD 14) # use c++ 14
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/dependencies.cmake)
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install CACHE PATH "Installation dir" FORCE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-Wall -Wextra -pedantic -Werror)

add_subdirectory(src)

install(
  DIRECTORY 
    test_images
    test_videos 
  DESTINATION 
    ${CMAKE_INSTALL_BINDIR}
)