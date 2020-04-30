cmake_minimum_required (VERSION 3.5)
project(PARTICLE_FILTER VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(sources src/particle_filter.cpp src/main.cpp ${HEADERS} ${HEADERS_HPP})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(particle_filter "")
target_include_directories(particle_filter
  SYSTEM
  PRIVATE
  src/system_include
)

target_sources(particle_filter
  PRIVATE
  src/helper_functions.h
  src/main.cpp
  src/map.h
  src/map.cpp
  src/particle_filter.cpp
  src/particle_filter.hpp
)

target_link_libraries(particle_filter z ssl uv uWS)

