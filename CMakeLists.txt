cmake_minimum_required(VERSION 2.8.3)
project(phasespace)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf roscpp genmsg)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(PkgConfig)

add_message_files(
  FILES
  PhaseSpace.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phasespace 
    CATKIN_DEPENDS geometry_msgs tf
)
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    )

link_directories(${catkin_LIBRARY_DIRS})


add_library(phasespace 
  src/phaseSpaceDriver.cpp
  src/udpServer.cpp)
add_dependencies(phasespace 
  phasespace_gencpp)

target_link_libraries(phasespace
  ${catkin_LIBRARIES}
)

add_executable(phaseSpace_print 
  tools/phaseSpace_print.cpp)
add_dependencies(phaseSpace_print 
  phasespace)
target_link_libraries(phaseSpace_print 
  phasespace 
  ${catkin_LIBRARIES}
)




