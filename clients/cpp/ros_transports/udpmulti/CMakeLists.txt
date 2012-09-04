project(udpmulti)

find_package(ROS REQUIRED COMPONENTS
  catkin genmsg
  cpp_common rostime rosconsole roscpp_serialization
  roscpp_traits std_msgs rosgraph_msgs xmlrpcpp rosunit
  pluginlib roscpp
  )
include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include)

include(CheckIncludeFiles)
include(CheckFunctionExists)



add_message_files(
  DIRECTORY msg
  FILES UDPMultTopic.msg
)

add_service_files(
  DIRECTORY srv
  FILES UDPMultClearAll.srv  UDPMultGetTopicList.srv  UDPMultRegisterTopic.srv
)

#uncomment if you have defined messages
generate_messages()
#uncomment if you have defined services
# generate_services()

catkin_project(${PROJECT_NAME}
    LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS cpp_common rostime rosconsole roscpp_serialization
          roscpp_traits std_msgs rosgraph_msgs xmlrpcpp roscpp
  )



#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} 
    src/udpmulti_transport_plugin.cpp
    src/udpmulti_publisher_link.cpp
    src/udpmulti_subscriber_link.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${ROS_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(udpmulti_manager src/udpmulti_manager.cpp)
target_link_libraries(udpmulti_manager ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION share/${PROJECT_NAME}/lib
    LIBRARY DESTINATION share/${PROJECT_NAME}/lib
 )
install(TARGETS udpmulti_manager DESTINATION bin)
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})
install(FILES udpmulti_plugin.xml DESTINATION share/${PROJECT_NAME})
