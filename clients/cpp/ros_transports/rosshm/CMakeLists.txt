project(rosshm)

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
  FILES SharedMemBlock.msg  SharedMemHeader.msg
)

add_service_files(
  DIRECTORY srv
  FILES SHMClearAll.srv  SHMGetBlocks.srv  SHMReleaseMemory.srv
)

#uncomment if you have defined messages
generate_messages()
#uncomment if you have defined services
# generate_services()

catkin_project(rosshm
  LIBRARIES rosshm
  INCLUDE_DIRS include
  DEPENDS cpp_common rostime rosconsole roscpp_serialization
          roscpp_traits std_msgs rosgraph_msgs xmlrpcpp roscpp
  )


#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} 
    src/SharedMemoryBlock.cpp
    src/SharedMemoryBlockDescriptor.cpp
    src/shm_transport_plugin.cpp
    src/shm_publisher_link.cpp
    src/shm_subscriber_link.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${ROS_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(sharedmem_manager src/sharedmem_manager.cpp)
target_link_libraries(sharedmem_manager ${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION share/${PROJECT_NAME}/lib
    LIBRARY DESTINATION share/${PROJECT_NAME}/lib
 )
install(TARGETS sharedmem_manager DESTINATION bin)
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})
install(FILES rosshm_plugin.xml DESTINATION share/${PROJECT_NAME})
