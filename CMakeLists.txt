cmake_minimum_required(VERSION 3.16)
project(arap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++${CMAKE_CXX_STANDARD}")

# enable_testing()

include_directories(src)

add_subdirectory(src)
# add_subdirectory(tests)
