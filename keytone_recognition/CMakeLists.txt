CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( keytone_recognition )

SET(DCMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  roboy_communication_cognition
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

ADD_EXECUTABLE( keytone_recognition ${incs} src/main_microphone.cpp src/freqOmeter.cpp)

##  !!!! DON'T EVER FUCKING FORGET THIS AGAIN !!!!
add_dependencies(keytone_recognition awesome_svcs_generate_messages_cpp) # awesome_msgs_generate_messages_cpp

TARGET_LINK_LIBRARIES(keytone_recognition ${catkin_LIBRARIES} -lfftw3f -lncurses)
