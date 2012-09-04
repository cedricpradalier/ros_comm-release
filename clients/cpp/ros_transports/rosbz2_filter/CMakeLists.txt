project(rosbz2_filter)

find_package(ROS REQUIRED COMPONENTS
  catkin genmsg
  cpp_common rostime rosconsole roscpp_serialization
  roscpp_traits std_msgs rosgraph_msgs xmlrpcpp rosunit
  pluginlib roscpp
  )
include_directories(${BZIP2_INCLUDE_DIR} ${ROS_INCLUDE_DIRS})
find_package(BZip2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include)

include(CheckIncludeFiles)
include(CheckFunctionExists)

catkin_project(${PROJECT_NAME}
    LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS cpp_common rostime rosconsole roscpp_serialization
          roscpp_traits std_msgs rosgraph_msgs xmlrpcpp roscpp
  )



#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/bz2_transport_filter_plugin.cpp)
target_link_libraries(${PROJECT_NAME}
    ${BZIP2_LIBRARIES} ${ROS_LIBRARIES} ${Boost_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION share/${PROJECT_NAME}/lib
    LIBRARY DESTINATION share/${PROJECT_NAME}/lib
 )
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})
install(FILES rosbz2_plugin.xml DESTINATION share/${PROJECT_NAME})
