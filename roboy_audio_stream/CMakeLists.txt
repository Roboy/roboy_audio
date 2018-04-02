cmake_minimum_required(VERSION 2.8.3)

project(roboy_audio_stream)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roboy_communication_cognition
)
find_package(Threads)
find_library (FFTW_LIBRARIES NAMES fftw3f )
FIND_LIBRARY(WIRINGPI_LIB NAMES wiringPi)
FIND_LIBRARY(WIRINGPI_DEV_LIB NAMES wiringPiDev)
FIND_LIBRARY(CRYPT_LIB NAMES crypt)
find_library(MATRIX_HAL_LIBRARIES NAMES matrix_creator_hal)
FIND_LIBRARY(GFLAGS_LIB NAMES gflags)
message(STATUS "gflags found =>" "${GFLAGS_LIB}") 

set(MATRIX_CREATOR_LIBRARIES ${MATRIX_HAL_LIBRARIES} ${FFTW_LIBRARIES} ${WIRINGPI_LIB})
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(roboy_audio_capture src/audio_capture.cpp)
add_executable(roboy_stream src/miccapture.cpp)
target_link_libraries(roboy_audio_capture ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(roboy_stream ${catkin_LIBRARIES} ${MATRIX_CREATOR_LIBRARIES} ${CRYPT_LIB} ${WIRINGPI_DEV_LIB} ${CMAKE_THREAD_LIBS_INIT} ${GFLAGS_LIB})
add_dependencies(roboy_audio_capture ${catkin_EXPORTED_TARGETS})

install(TARGETS roboy_audio_capture
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
