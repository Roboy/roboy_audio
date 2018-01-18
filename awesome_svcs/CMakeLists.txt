cmake_minimum_required(VERSION 2.8.3)
project(awesome_svcs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
	/usr/local/lib/cmake 
	/opt/ros/indigo/share/cmake_modules/cmake/Modules 
	${PROJECT_SOURCE_DIR}/cmake
)

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
	DIRECTORY
	srv
	FILES
# vision services
	locateStuff.srv
# control services
	moveJoint.srv
	moveJoints.srv
	moveJointSequence.srv
	moveJointsSequence.srv
	moveEndeffector.srv
	enableStiffness.srv
	disableStiffness.srv
	setHandOpen.srv
	goToPosture.srv
	sayAloud.srv
	walkTo.srv
	queryJoints.srv
	cameraFrames.srv
	keyboardKeys.srv
# audio services
	whichTones.srv
	setLeds.srv
	qualityOfTone.srv
	qualityOfTones.srv
# music services
	selfCalibration.srv
	reachabilityMap.srv
)

generate_messages(
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime message_generation)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Don't Forget This! I already told you once!
#add_dependencies(control_naoqi control_naoqi_generate_messages_cpp awesome_msgs_generate_messages_cpp)





