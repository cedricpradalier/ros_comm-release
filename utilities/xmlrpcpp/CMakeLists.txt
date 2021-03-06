cmake_minimum_required(VERSION 2.8)
project(xmlrpcpp)
catkin_project(xmlrpcpp
  INCLUDE_DIRS include
  LIBRARIES xmlrpcpp
  )

find_package(catkin REQUIRED COMPONENTS cpp_common)
include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp 
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  )

if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()

message("TODO: consider moving xmlrpcpp's headers into a subdirectory")
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

#explicitly install library and includes
install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
 )
