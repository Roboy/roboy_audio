cmake_minimum_required(VERSION 2.8.3)
project(doa_estimation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    doa_estimation_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/direction_of_arrival_demo.cpp
    src/direction_of_arrival.cpp
    src/cross_correlation.cpp
    src/matrix_driver.cpp
    src/microphone_array.cpp
    src/everloop.cpp
    src/wishbone_bus.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    -lfftw3
    -lfftw3f
    -lgflags
    -lwiringPi
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
