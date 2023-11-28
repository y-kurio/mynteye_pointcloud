cmake_minimum_required(VERSION 3.0.2)
project(mynteye_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  pointData.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_cd
  src/point/constructor.cpp
  src/point/method.cpp
  src/point/debug_main.cpp
  src/point/property.cpp
)
add_executable(${PROJECT_NAME}_rp
  src/Riskprediction/constructor.cpp
  src/Riskprediction/method.cpp
  src/Riskprediction/debug_main.cpp
  src/Riskprediction/property.cpp
)

add_dependencies(${PROJECT_NAME}_cd ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_rp ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_cd ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_rp ${catkin_LIBRARIES} )

add_definitions(-std=c++14)
