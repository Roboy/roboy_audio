CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT ( keytone_recognition )

SET(DCMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  roboy_communication_cognition
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include/"
)

FILE(GLOB incs "include/*.hpp")
FILE(GLOB srcs "src/*.cpp")

ADD_EXECUTABLE( keytone_recognition ${incs} src/main_microphone.cpp src/freqOmeter.cpp)

##  !!!! DON'T EVER FUCKING FORGET THIS AGAIN !!!!
add_dependencies(keytone_recognition ${catkin_EXPORTED_TARGETS})

TARGET_LINK_LIBRARIES(keytone_recognition ${catkin_LIBRARIES} -lfftw3f -lncurses)
