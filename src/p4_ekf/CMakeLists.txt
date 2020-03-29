cmake_minimum_required (VERSION 3.5)
project(ExtendedKF VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions

add_compile_options(-Wall -Wextra -pedantic -Werror)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(ExtendedKF "")

target_include_directories(ExtendedKF
  SYSTEM
  PRIVATE
  src/system_include #this is actually ugly, but so is copying Eigen into our source tree.
)

target_sources(ExtendedKF
  PRIVATE
  src/main.cpp 
  src/tools.cpp 
  src/FusionEKF.cpp 
  src/tools.h 
  src/FusionEKF.h 
)

target_link_libraries(ExtendedKF z ssl uv uWS)
