cmake_minimum_required(VERSION 3.0.2)
project(c1t_razor_imu_m0_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cav_driver_utils
  cav_msgs
  razor_imu_m0_driver
  roscpp
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cav_driver_utils cav_msgs razor_imu_m0_driver roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/c1t_razor_imu_m0_driver.cpp
  src/c1t_razor_imu_m0_driver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
