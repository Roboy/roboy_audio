CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( nao_audio )

SET(DCMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  audio_common_msgs
  awesome_svcs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/"
)

#FIND_PACKAGE(Fftw3 REQUIRED)
#INCLUDE_DIRECTORIES(${FFTW3_INCLUDE_DIRS})

#FILE(GLOB incs "include/*.h")
FILE(GLOB incs "include/*.hpp")
FILE(GLOB srcs "src/*.cpp")

ADD_EXECUTABLE( nao_audio ${incs} src/main_microphone.cpp src/freqOmeter.cpp)

##  !!!! DON'T EVER FUCKING FORGET THIS AGAIN !!!!
add_dependencies(nao_audio awesome_msgs_generate_messages_cpp awesome_svcs_generate_messages_cpp)

TARGET_LINK_LIBRARIES(nao_audio ${catkin_LIBRARIES} -lfftw3f -lncurses)
