cmake_minimum_required(VERSION 2.8.3)
project(doa_estimation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    roboy_communication_cognition
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  Eigen
)

add_executable(${PROJECT_NAME}
    src/direction_of_arrival_demo.cpp
    src/direction_of_arrival.cpp
    src/cross_correlation.cpp
    src/matrix_driver.cpp
    src/microphone_array.cpp
    src/everloop.cpp
    src/wishbone_bus.cpp
    #src/music_spectrum.cpp
    )

add_executable(collect_data
    src/collect_data.cpp
    src/direction_of_arrival.cpp
    src/cross_correlation.cpp
    src/matrix_driver.cpp
    src/microphone_array.cpp
    src/everloop.cpp
    src/wishbone_bus.cpp
   # src/music_spectrum.cpp
    )



target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)
target_compile_features(collect_data PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    -lfftw3
    -lfftw3f
    -lgflags
    -lwiringPi
)
target_link_libraries(collect_data
        ${catkin_LIBRARIES}
	-lfftw3
	-lfftw3f
        -lgflags
        -lwiringPi
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(collect_data ${catkin_EXPORTED_TARGETS})
