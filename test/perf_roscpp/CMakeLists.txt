project(perf_roscpp)
catkin_project(perf_roscpp)
find_package(Boost COMPONENTS thread)
find_package(catkin COMPONENTS roscpp xmlrpcpp rosconsole)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  LatencyMessage.msg
  ThroughputMessage.msg
)
generate_messages(DEPENDENCIES)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} EXCLUDE_FROM_ALL src/intra.cpp src/inter.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-intra_suite EXCLUDE_FROM_ALL src/intra_suite.cpp)
target_link_libraries(${PROJECT_NAME}-intra_suite ${PROJECT_NAME})
if(TARGET tests)
  add_dependencies(tests ${PROJECT_NAME}-intra_suite)
endif()
