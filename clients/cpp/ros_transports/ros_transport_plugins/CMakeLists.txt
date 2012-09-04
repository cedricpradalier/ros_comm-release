project(ros_transport_plugins)

find_package(ROS REQUIRED COMPONENTS
  catkin genmsg
  cpp_common rostime rosconsole roscpp_serialization
  roscpp_traits std_msgs rosgraph_msgs xmlrpcpp rosunit
  pluginlib
  )
include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include)

include(CheckIncludeFiles)
include(CheckFunctionExists)



add_executable(list_all_transports src/list_all_transports.cpp)
target_link_libraries(list_all_transports roscpp
  ${ROS_LIBRARIES} ${Boost_LIBRARIES})

add_executable(subscribe_with_hints tests/subscribe_with_hints.cpp)
target_link_libraries(subscribe_with_hints roscpp
  ${ROS_LIBRARIES} ${Boost_LIBRARIES})

add_executable(publish_with_hints tests/publish_with_hints.cpp)
target_link_libraries(publish_with_hints roscpp
  ${ROS_LIBRARIES} ${Boost_LIBRARIES})


install(FILES manifest.xml DESTINATION share/ros_transport_plugins)
install(TARGETS list_all_transports DESTINATION bin)
