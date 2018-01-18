cmake_minimum_required(VERSION 2.8.3)

project(roboy_audio_capture)

find_package(catkin REQUIRED COMPONENTS roscpp roboy_communication_cognition)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(roboy_audio_capture src/audio_capture.cpp)
target_link_libraries(roboy_audio_capture ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roboy_audio_capture ${catkin_EXPORTED_TARGETS})

install(TARGETS roboy_audio_capture
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
