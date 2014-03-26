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
  PhaseSpaceMsg.msg
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
 ${CMAKE_CURRENT_BINARY_DIR}/include 
                    )

link_directories(${catkin_LIBRARY_DIRS})


add_library(phasespace 
  src/phaseSpaceDriver.cpp
  src/udpServer.cpp)
add_dependencies(phasespace 
  owlsock
  phasespace_gencpp)

target_link_libraries(phasespace
  owlsock
  ${catkin_LIBRARIES}
)

get_filename_component(PHASESPACE_SRC_FOLDER "model/" ABSOLUTE)
configure_file(include/src_folder.h.in include/src_folder.h)

add_executable(phasespace_print 
  tools/phasespace_print.cpp)
add_dependencies(phasespace_print 
  phasespace)
target_link_libraries(phasespace_print 
  phasespace 
  ${catkin_LIBRARIES}
)

add_executable(phasespace_node 
  nodes/phasespace_node.cpp)
add_dependencies(phasespace_node 
  phasespace)

target_link_libraries(phasespace_node
  phasespace
  ${catkin_LIBRARIES}
)

add_executable(phasespace_jim 
  tools/track2.cc)
add_dependencies(phasespace_jim
  phasespace)
target_link_libraries(phasespace_jim
  phasespace
  ${catkin_LIBRARIES}
)

install(TARGETS phasespace_print
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS phasespace_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS phasespace
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



