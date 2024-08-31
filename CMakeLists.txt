cmake_minimum_required(VERSION 3.22)

project(HAL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)
FetchContent_MakeAvailable(moteus)

find_package(Eigen3 REQUIRED)

include_directories(
    ${Eigen_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    /usr/include/libserial 
)

set(LIBSERIAL_LIB "/usr/lib/x86_64-linux-gnu/libserial.so")
set(SOURCES 
    src/UM7.cc 
    src/hal_bringup.cc 
    src/hal.cc
)

add_executable(hal_bringup ${SOURCES})
target_link_libraries(hal_bringup moteus::cpp pthread ${LIBSERIAL_LIB})

