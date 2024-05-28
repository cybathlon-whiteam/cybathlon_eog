cmake_minimum_required(VERSION 2.8.3)
project(cybathlon_eog)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs
			 dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

generate_dynamic_reconfigure_options(
	cfg/EogDetector.cfg
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
		    include
		    
)

add_definitions(${EIGEN3_DEFINITIONS})


add_library(${PROJECT_NAME}
	src/EogDetector.cpp
)

target_link_libraries(${PROJECT_NAME} 
		     ${catkin_LIBRARIES} 
			 ${Eigen3_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
	         ${catkin_EXPORTED_TARGETS}
)


#################
## Executables ##
#################

add_executable(eog_detector src/eog_detector.cpp)
target_link_libraries(eog_detector ${PROJECT_NAME})
add_dependencies(eog_detector ${PROJECT_NAME}_gencfg)


